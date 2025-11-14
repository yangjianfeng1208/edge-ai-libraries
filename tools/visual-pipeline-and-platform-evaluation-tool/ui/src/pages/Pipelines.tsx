import { useParams } from "react-router";
import {
  useGetPipelineQuery,
  useRunPipelineMutation,
  useStopPipelineInstanceMutation,
} from "@/api/api.generated";
import {
  type Edge as ReactFlowEdge,
  type Node as ReactFlowNode,
} from "@xyflow/react";
import { useState } from "react";
import PipelineEditor from "@/features/pipeline-editor/PipelineEditor.tsx";
import FpsDisplay from "@/features/pipeline-editor/FpsDisplay.tsx";
import { toast } from "sonner";
import RunPipelineButton from "@/features/pipeline-editor/RunPipelineButton.tsx";
import StopPipelineButton from "@/features/pipeline-editor/StopPipelineButton.tsx";
import StatePreviewButton from "@/features/pipeline-editor/StatePreviewButton.tsx";

type UrlParams = {
  id: string;
};

const Pipelines = () => {
  const { id } = useParams<UrlParams>();
  const [pipelineInstanceId, setPipelineInstanceId] = useState<string | null>(
    null,
  );
  const [currentNodes, setCurrentNodes] = useState<ReactFlowNode[]>([]);
  const [currentEdges, setCurrentEdges] = useState<ReactFlowEdge[]>([]);

  const { data, isSuccess } = useGetPipelineQuery(
    {
      name: "predefined_pipelines",
      version: id ?? "",
    },
    {
      skip: !id,
    },
  );

  const [runPipeline, { isLoading: isRunning }] = useRunPipelineMutation();
  const [stopPipelineInstance, { isLoading: isStopping }] =
    useStopPipelineInstanceMutation();

  const handleNodesChange = (nodes: ReactFlowNode[]) => {
    setCurrentNodes(nodes);
  };

  const handleEdgesChange = (edges: ReactFlowEdge[]) => {
    setCurrentEdges(edges);
  };

  const handleRunPipeline = async () => {
    if (!id) return;

    try {
      const apiNodes = currentNodes.map((node) => ({
        id: node.id,
        type: node.type ?? "default",
        data: Object.fromEntries(
          Object.entries(node.data ?? {}).map(([key, value]) => [
            key,
            String(value),
          ]),
        ),
      }));

      const response = await runPipeline({
        name: "predefined_pipelines",
        version: id,
        pipelineRequestRunInput: {
          async_: true,
          source: {
            type: "uri",
            uri: "https://storage.openvinotoolkit.org/repositories/openvino_notebooks/data/data/video/people.mp4",
          },
          parameters: {
            inferencing_channels: 20,
            recording_channels: 0,
            pipeline_graph: {
              nodes: apiNodes,
              edges: currentEdges,
            },
          },
          tags: {
            additionalProp1: "string",
          },
        },
      }).unwrap();

      if (
        response &&
        typeof response === "object" &&
        "instance_id" in response
      ) {
        setPipelineInstanceId(response.instance_id as string);
      }

      toast.success("Pipeline run started", {
        description: new Date().toISOString(),
      });
    } catch (error) {
      toast.error("Failed to start pipeline", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
      console.error("Failed to start pipeline:", error);
    }
  };

  const handleStopPipeline = async () => {
    if (!pipelineInstanceId) return;

    try {
      await stopPipelineInstance({
        instanceId: pipelineInstanceId,
      }).unwrap();

      setPipelineInstanceId(null);

      toast.success("Pipeline stopped", {
        description: new Date().toISOString(),
      });
    } catch (error) {
      toast.error("Failed to stop pipeline", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
      console.error("Failed to stop pipeline:", error);
    }
  };

  if (isSuccess) {
    return (
      <div style={{ width: "100%", height: "100vh", position: "relative" }}>
        <PipelineEditor
          pipelineData={data}
          onNodesChange={handleNodesChange}
          onEdgesChange={handleEdgesChange}
        />

        <div className="absolute top-4 right-4 z-10 flex flex-col gap-2 items-end">
          <FpsDisplay />

          <div className="flex gap-2">
            {pipelineInstanceId ? (
              <StopPipelineButton
                isStopping={isStopping}
                onStopPipeline={handleStopPipeline}
              />
            ) : (
              <RunPipelineButton
                isRunning={isRunning}
                onRunPipeline={handleRunPipeline}
              />
            )}

            <StatePreviewButton edges={currentEdges} nodes={currentNodes} />
          </div>
        </div>
      </div>
    );
  }

  return <div>Loading pipeline: {id}</div>;
};

export default Pipelines;
