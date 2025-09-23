# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import os
from sqlalchemy.ext.asyncio import create_async_engine
from langchain.globals import set_verbose
from langchain.callbacks import streaming_stdout
from langchain_postgres.vectorstores import PGVector as EGAIVectorDB
from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import (
    RunnableParallel,
    RunnablePassthrough,
    RunnableLambda,
)
from langchain_core.vectorstores import VectorStoreRetriever as EGAIVectorStoreRetriever
from langchain_openai import ChatOpenAI as EGAIModelServing
from langchain_openai import OpenAIEmbeddings as EGAIEmbeddings
from .custom_reranker import CustomReranker
import logging
from opentelemetry import trace
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
import openlit

set_verbose(True)

logging.basicConfig(level=logging.INFO)

# Check if OTLP endpoint is set in environment variables
otlp_endpoint = os.environ.get("OTLP_ENDPOINT", False)

# Initialize OpenTelemetry
if not isinstance(trace.get_tracer_provider(), TracerProvider):
    tracer_provider = TracerProvider()
    trace.set_tracer_provider(tracer_provider)

    # Set up OTLP exporter and span processor
    if not otlp_endpoint:
        logging.warning(
            "No OTLP endpoint provided - Telemetry data will not be collected."
        )
    else:
        otlp_exporter = OTLPSpanExporter()
        span_processor = BatchSpanProcessor(otlp_exporter)
        tracer_provider.add_span_processor(span_processor)

        openlit.init(
            otlp_endpoint=otlp_endpoint,
            application_name=os.environ.get("OTEL_SERVICE_NAME", "chatqna"),
            environment=os.environ.get("OTEL_SERVICE_ENV", "chatqna"),
        )

        logging.info(
            f"Tracing enabled: OpenTelemetry configured using OTLP endpoint at {otlp_endpoint}"
        )

PG_CONNECTION_STRING = os.getenv("PG_CONNECTION_STRING")
MODEL_NAME = os.getenv("EMBEDDING_MODEL", "Alibaba-NLP/gte-large-en-v1.5")
EMBEDDING_ENDPOINT_URL = os.getenv("EMBEDDING_ENDPOINT_URL", "http://localhost:6006")
COLLECTION_NAME = os.getenv("INDEX_NAME")
FETCH_K = int(os.getenv("FETCH_K", "10"))

engine = create_async_engine(PG_CONNECTION_STRING)

# Init Embeddings via Intel Edge GenerativeAI Suite
try:
    embedder = EGAIEmbeddings(
        openai_api_key="EMPTY",
        openai_api_base="{}".format(EMBEDDING_ENDPOINT_URL),
        model=MODEL_NAME
    )
    logging.info(
        f"Embeddings initialized with endpoint configured in EMBEDDING_ENDPOINT_URL"
    )
except Exception as e:
    logging.error(f"Failed to initialize embeddings: {str(e)}")
    raise

knowledge_base = EGAIVectorDB(
    embeddings=embedder,
    collection_name=COLLECTION_NAME,
    connection=engine,
)
retriever = EGAIVectorStoreRetriever(
    vectorstore=knowledge_base,
    search_type="mmr",
    search_kwargs={"k": FETCH_K, "fetch_k": FETCH_K * 3},
)

# Define our prompt
template = """
Use the following pieces of context from retrieved
dataset and prior conversation history to answer the question. 
Do not make up an answer if there is no context provided to help answer it.

Conversation history:
---------
{history}
---------

Context:
---------
{context}

---------
Question: {question}
---------

Answer:
"""


prompt = ChatPromptTemplate.from_template(template)

ENDPOINT_URL = os.getenv("ENDPOINT_URL", "http://localhost:8080")

# Check which LLM inference backend is being used
LLM_BACKEND = None
if "ovms" in ENDPOINT_URL.lower():
    LLM_BACKEND = "ovms"
elif "text-generation" in ENDPOINT_URL.lower():
    LLM_BACKEND = "text-generation"
elif "vllm" in ENDPOINT_URL.lower():
    LLM_BACKEND = "vllm"
else:
    LLM_BACKEND = "unknown"

logging.info(f"Using LLM inference backend: {LLM_BACKEND}")
LLM_MODEL = os.getenv("LLM_MODEL", "Intel/neural-chat-7b-v3-3")
RERANKER_ENDPOINT = os.getenv("RERANKER_ENDPOINT", "http://localhost:9090/rerank")
callbacks = [streaming_stdout.StreamingStdOutCallbackHandler()]

async def context_retriever_fn(chain_inputs: dict):
    question = chain_inputs.get("question", "")
    if not question:
        return {}   # to keep shape consistent

    retrieved_docs = await retriever.aget_relevant_documents(question)
    return retrieved_docs     # context: list[Document]

# Format the context in a readable way
def format_docs(docs):
    if not docs:
        return "No relevant context found."
    
    formatted_docs = []
    for i, doc in enumerate(docs, 1):
        content = doc.page_content.strip()
        metadata = doc.metadata or {}
        source = metadata.get("source", "Unknown source")
        
        formatted_docs.append(f"[Document {i}] {content}\nSource: {source}")
    
    return "\n\n".join(formatted_docs)

async def process_chunks(conversation_messages, max_tokens):
    print(f"Inside process_chunks, conversation_messages: {conversation_messages}, max_tokens: {max_tokens}")
    # Combine all messages for context
    if len(conversation_messages) > 1:
        history = "\n".join([f"{msg.role}: {msg.content}" for msg in conversation_messages[:-1]])
    else:
        history = ""
    print(f"history inside process_chunks: {history}")

    question_text = conversation_messages[-1].content
    if not question_text or not question_text.strip():
        raise ValueError("Question text cannot be empty")

    context_retriever = RunnableLambda(context_retriever_fn) # it passes all chain_input dict to context_retriever fn

    if LLM_BACKEND in ["vllm", "unknown"]:
        seed_value = None
        model = EGAIModelServing(
            openai_api_key="EMPTY",
            openai_api_base="{}".format(ENDPOINT_URL),
            model_name=LLM_MODEL,
            top_p=0.99,
            temperature=0.01,
            streaming=True,
            callbacks=callbacks,
            stop=["\n\n"],
        )
    else:
        seed_value = int(os.getenv("SEED", 42))
        model = EGAIModelServing(
            openai_api_key="EMPTY",
            openai_api_base="{}".format(ENDPOINT_URL),
            model_name=LLM_MODEL,
            top_p=0.99,
            temperature=0.01,
            streaming=True,
            callbacks=callbacks,
            seed=seed_value,
            max_tokens=max_tokens
        )

    re_ranker = CustomReranker(reranking_endpoint=RERANKER_ENDPOINT)
    re_ranker_lambda = RunnableLambda(re_ranker.rerank)

    print(f"Final Prompt: {prompt}")

    # RAG Chain
    chain = (
    RunnableParallel({
        "context": context_retriever,  # context retrieved from vector store
        "question": lambda x: x["question"],  # passes through the question
        "history": lambda x: x["history"]  # passes through the history
    })
    | re_ranker_lambda
    | {"context": (lambda x: format_docs(x["context"])), "question": lambda x: x["question"]}
    | prompt
    | model
    | StrOutputParser()
    )

    # Run the chain with all inputs
    chain_input = {
        "history": history, # chain will call context_retriever internally, no need to add it.
        "question": question_text
    }

    async for log in chain.astream(chain_input):
        yield f"data: {log}\n\n"

