import { useParams } from "react-router";
import {
  useGetPipelineQuery,
  useRunPipelineMutation,
  useStopPipelineInstanceMutation,
} from "@/api/api.generated";
import {
  type Edge as ReactFlowEdge,
  type Node as ReactFlowNode,
  type Viewport,
} from "@xyflow/react";
import { useState } from "react";
import PipelineEditor from "@/features/pipeline-editor/PipelineEditor.tsx";
import FpsDisplay from "@/features/pipeline-editor/FpsDisplay.tsx";
import { toast } from "sonner";
import RunPipelineButton from "@/features/pipeline-editor/RunPipelineButton.tsx";
import StopPipelineButton from "@/features/pipeline-editor/StopPipelineButton.tsx";
import StatePreviewButton from "@/features/pipeline-editor/StatePreviewButton.tsx";
import ExportPipelineButton from "@/features/pipeline-editor/ExportPipelineButton.tsx";
import ImportPipelineButton from "@/features/pipeline-editor/ImportPipelineButton.tsx";

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
  const [currentViewport, setCurrentViewport] = useState<Viewport>({
    x: 0,
    y: 0,
    zoom: 1,
  });
  const [editorKey, setEditorKey] = useState(0);
  const [shouldFitView, setShouldFitView] = useState(false);

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

  const handleViewportChange = (viewport: Viewport) => {
    setCurrentViewport(viewport);
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

  const handleImport = (
    nodes: ReactFlowNode[],
    edges: ReactFlowEdge[],
    viewport: Viewport,
    shouldFitView: boolean,
  ) => {
    setCurrentNodes(nodes);
    setCurrentEdges(edges);
    setCurrentViewport(viewport);
    setShouldFitView(shouldFitView);
    setEditorKey((prev) => prev + 1); // Force PipelineEditor to re-initialize
  };

  if (isSuccess && data) {
    return (
      <div className="w-full h-screen relative">
        <PipelineEditor
          key={editorKey}
          pipelineData={data}
          onNodesChange={handleNodesChange}
          onEdgesChange={handleEdgesChange}
          onViewportChange={handleViewportChange}
          initialNodes={currentNodes.length > 0 ? currentNodes : undefined}
          initialEdges={currentEdges.length > 0 ? currentEdges : undefined}
          initialViewport={
            currentNodes.length > 0 ? currentViewport : undefined
          }
          shouldFitView={shouldFitView}
        />

        <div className="absolute top-4 right-4">
          <FpsDisplay />
        </div>

        <div className="absolute top-4 left-4 z-10 flex flex-col gap-2 items-end">
          <div className="flex gap-2">
            <ImportPipelineButton onImport={handleImport} />

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

            <ExportPipelineButton
              edges={currentEdges}
              nodes={currentNodes}
              viewport={currentViewport}
              pipelineName={data.version}
            />

            <StatePreviewButton
              edges={currentEdges}
              nodes={currentNodes}
              viewport={currentViewport}
            />
          </div>
        </div>
      </div>
    );
  }

  return <div>Loading pipeline: {id}</div>;
};

export default Pipelines;
