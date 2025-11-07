import {
  Background,
  BackgroundVariant,
  Controls,
  type Edge as ReactFlowEdge,
  type Node as ReactFlowNode,
  type NodeMouseHandler,
  ReactFlow,
  useEdgesState,
  useNodesState,
} from "@xyflow/react";
import type { Node } from "@/api/api.generated.ts";
import "@xyflow/react/dist/style.css";
import { useEffect, useState } from "react";
import { nodeTypes } from "@/features/pipeline-editor/nodes";
import NodeDataPanel from "@/features/pipeline-editor/NodeDataPanel.tsx";
import { type Pipeline } from "@/api/api.generated";
import {
  createGraphLayout,
  LayoutDirection,
} from "@/features/pipeline-editor/utils/graphLayout";

interface PipelineEditorProps {
  pipelineData?: Pipeline;
  onNodesChange?: (nodes: ReactFlowNode[]) => void;
  onEdgesChange?: (edges: ReactFlowEdge[]) => void;
}

const PipelineEditor = ({
  pipelineData,
  onNodesChange: onNodesChangeCallback,
  onEdgesChange: onEdgesChangeCallback,
}: PipelineEditorProps) => {
  const [selectedNode, setSelectedNode] = useState<ReactFlowNode | null>(null);
  const [nodes, setNodes, onNodesChange] = useNodesState<ReactFlowNode>([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState<ReactFlowEdge>([]);

  const onNodeClick: NodeMouseHandler = (event, node) => {
    event.stopPropagation();
    setSelectedNode(node);
  };

  const onPaneClick = () => {
    setSelectedNode(null);
  };

  const handleNodeDataUpdate = (
    nodeId: string,
    updatedData: Record<string, unknown>,
  ) => {
    setNodes((currentNodes) => {
      const updatedNodes = currentNodes.map((node) =>
        node.id === nodeId ? { ...node, data: updatedData } : node,
      );
      onNodesChangeCallback?.(updatedNodes);
      return updatedNodes;
    });
  };

  useEffect(() => {
    onNodesChangeCallback?.(nodes);
  }, [nodes, onNodesChangeCallback]);

  useEffect(() => {
    onEdgesChangeCallback?.(edges);
  }, [edges, onEdgesChangeCallback]);

  useEffect(() => {
    if (pipelineData?.launch_config) {
      // const nodes = pipelineData.launch_config.nodes || [];
      // const edges = pipelineData.launch_config.edges || [];

      // Get the raw nodes and edges from API (using hardcoded data for now)
      const nodes: Node[] = [
        {
          id: "0",
          type: "filesrc",
          data: {
            location: "${VIDEO}",
          },
        },
        {
          id: "1",
          type: "qtdemux",
          data: {},
        },
        {
          id: "2",
          type: "h264parse",
          data: {},
        },
        {
          id: "3",
          type: "vah264dec",
          data: {},
        },
        {
          id: "4",
          type: "video/x-raw(memory:VAMemory)",
          data: {},
        },
        {
          id: "5",
          type: "gvafpscounter",
          data: {
            "starting-frame": "500",
          },
        },
        {
          id: "6",
          type: "gvadetect",
          data: {
            model: "${YOLO11n_POST_MODEL}",
            device: "GPU",
            "pre-process-backend": "va-surface-sharing",
            "model-instance-id": "yolo11-pose",
          },
        },
        {
          id: "7",
          type: "queue2",
          data: {},
        },
        {
          id: "8",
          type: "gvatrack",
          data: {
            "tracking-type": "short-term-imageless",
          },
        },
        {
          id: "9",
          type: "gvawatermark",
          data: {},
        },
        {
          id: "10",
          type: "gvametaconvert",
          data: {
            format: "json",
            "json-indent": "4",
          },
        },
        {
          id: "11",
          type: "gvametapublish",
          data: {
            method: "file",
            "file-path": "/dev/null",
          },
        },
        {
          id: "12",
          type: "fakesink",
          data: {},
        },
      ];

      const edges: ReactFlowEdge[] = [
        {
          id: "0",
          source: "0",
          target: "1",
        },
        {
          id: "1",
          source: "1",
          target: "2",
        },
        {
          id: "2",
          source: "2",
          target: "3",
        },
        {
          id: "3",
          source: "3",
          target: "4",
        },
        {
          id: "4",
          source: "4",
          target: "5",
        },
        {
          id: "5",
          source: "5",
          target: "6",
        },
        {
          id: "6",
          source: "6",
          target: "7",
        },
        {
          id: "7",
          source: "7",
          target: "8",
        },
        {
          id: "8",
          source: "8",
          target: "9",
        },
        {
          id: "9",
          source: "9",
          target: "10",
        },
        {
          id: "10",
          source: "10",
          target: "11",
        },
        {
          id: "11",
          source: "11",
          target: "12",
        },
      ];

      const transformedNodes = nodes.map(
        (node) =>
          ({
            ...node,
            type: node.type,
          }) as ReactFlowNode,
      );

      const nodesWithPositions = createGraphLayout(
        transformedNodes,
        edges,
        LayoutDirection.LeftToRight,
      );

      setNodes(nodesWithPositions);
      setEdges(edges);
    }
  }, [pipelineData, setNodes, setEdges]);

  return (
    <div style={{ width: "100%", height: "100vh", position: "relative" }}>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        nodeTypes={nodeTypes}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onNodeClick={onNodeClick}
        onPaneClick={onPaneClick}
        nodesDraggable={true}
        fitView
      >
        <Controls />
        <Background variant={BackgroundVariant.Dots} gap={12} size={1} />
      </ReactFlow>

      <NodeDataPanel
        selectedNode={selectedNode}
        onNodeDataUpdate={handleNodeDataUpdate}
      />
    </div>
  );
};

export default PipelineEditor;
