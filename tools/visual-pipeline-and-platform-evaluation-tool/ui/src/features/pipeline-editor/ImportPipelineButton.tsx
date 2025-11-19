import { useToGraphMutation } from "@/api/api.generated";
import { isApiError } from "@/lib/apiUtils";
import type { Edge, Node, Viewport } from "@xyflow/react";
import { FolderOpen } from "lucide-react";
import { useRef } from "react";
import { toast } from "sonner";
import { createGraphLayout } from "./utils/graphLayout";

type ImportPipelineButtonProps = {
  onImport: (
    nodes: Node[],
    edges: Edge[],
    viewport: Viewport,
    shouldFitView: boolean,
  ) => void;
};

const ImportPipelineButton = ({ onImport }: ImportPipelineButtonProps) => {
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [toGraph, { isLoading }] = useToGraphMutation();

  const handleClick = () => {
    fileInputRef.current?.click();
  };

  const handleFileChange = async (
    event: React.ChangeEvent<HTMLInputElement>,
  ) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const fileExtension = file.name.split(".").pop()?.toLowerCase();

    if (fileExtension !== "json" && fileExtension !== "txt") {
      toast.error("Invalid file type", {
        description: "Please upload a .json or .txt file",
      });
      return;
    }

    try {
      const fileContent = await file.text();

      if (fileExtension === "json") {
        const parsedData = JSON.parse(fileContent);

        if (!parsedData.nodes || !parsedData.edges) {
          toast.error("Invalid JSON format", {
            description:
              "JSON file must contain 'nodes' and 'edges' properties",
          });
          return;
        }

        const viewport = parsedData.viewport || { x: 0, y: 0, zoom: 1 };
        onImport(parsedData.nodes, parsedData.edges, viewport, false);
        toast.success("Pipeline imported");
      } else if (fileExtension === "txt") {
        const response = await toGraph({
          pipelineDescription: {
            pipeline_description: fileContent,
          },
        }).unwrap();

        const nodesWithPositions = createGraphLayout(
          response.nodes.map((node) => ({
            id: node.id,
            type: node.type,
            data: node.data,
            position: { x: 0, y: 0 },
          })),
          response.edges,
        );

        const viewport = { x: 0, y: 0, zoom: 1 };
        onImport(nodesWithPositions, response.edges, viewport, true);
        toast.success("Pipeline imported");
      }
    } catch (error) {
      const errorMessage = isApiError(error)
        ? error.data.message
        : error instanceof Error
          ? error.message
          : "Unknown error";

      toast.error("Failed to import pipeline", {
        description: errorMessage,
      });
      console.error("Failed to import pipeline:", error);
    } finally {
      if (fileInputRef.current) {
        fileInputRef.current.value = "";
      }
    }
  };

  return (
    <>
      <input
        ref={fileInputRef}
        className="hidden"
        type="file"
        accept=".json,.txt"
        onChange={handleFileChange}
      />
      <button
        onClick={handleClick}
        disabled={isLoading}
        className="bg-blue-600 hover:bg-blue-700 text-white p-2 rounded-lg shadow-lg transition-colors disabled:opacity-50"
        title="Import Pipeline"
      >
        <FolderOpen className="w-5 h-5" />
      </button>
    </>
  );
};

export default ImportPipelineButton;
