import { useState } from "react";
import { useNavigate } from "react-router";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import {
  useCreatePipelineMutation,
  useToDescriptionMutation,
} from "@/api/api.generated";
import { toast } from "sonner";
import type { Pipeline } from "@/api/api.generated";

interface CopyPipelineButtonProps {
  pipeline: Pipeline;
  children: React.ReactNode;
}

const CopyPipelineButton = ({
  pipeline,
  children,
}: CopyPipelineButtonProps) => {
  const navigate = useNavigate();
  const [open, setOpen] = useState(false);
  const [name, setName] = useState(`${pipeline.name} (copy)`);
  const [description, setDescription] = useState(pipeline.description);

  const [createPipeline, { isLoading: isCreating }] =
    useCreatePipelineMutation();
  const [toDescription, { isLoading: isConverting }] =
    useToDescriptionMutation();

  const handleCopy = async () => {
    if (!name.trim()) {
      toast.error("Name is required");
      return;
    }

    try {
      // Step 1: Convert pipeline graph to description
      const descriptionResponse = await toDescription({
        pipelineGraph: pipeline.pipeline_graph,
      }).unwrap();

      // Step 2: Create new pipeline with the description
      const response = await createPipeline({
        pipelineDefinition: {
          name: name.trim(),
          description: description.trim(),
          source: "USER_CREATED",
          type: pipeline.type,
          pipeline_description: descriptionResponse.pipeline_description,
          parameters: pipeline.parameters,
        },
      }).unwrap();

      if (response.id) {
        setOpen(false);
        toast.success("Pipeline copied successfully");
        navigate(`/pipelines/${response.id}`);
      }
    } catch (error) {
      toast.error("Failed to copy pipeline", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
      console.error("Failed to copy pipeline:", error);
    }
  };

  const isLoading = isConverting || isCreating;

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>{children}</DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Copy Pipeline</DialogTitle>
        </DialogHeader>
        <div className="space-y-4">
          <div>
            <label htmlFor="name" className="block text-sm font-medium mb-2">
              Name
            </label>
            <input
              id="name"
              type="text"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="Enter pipeline name..."
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>

          <div>
            <label
              htmlFor="description"
              className="block text-sm font-medium mb-2"
            >
              Description
            </label>
            <textarea
              id="description"
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              placeholder="Enter pipeline description..."
              className="w-full h-24 px-3 py-2 border rounded-md resize-none"
            />
          </div>

          <div className="flex justify-end gap-2">
            <button
              className="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 transition-colors"
              onClick={() => {
                setOpen(false);
                setName(`${pipeline.name} (copy)`);
                setDescription(pipeline.description);
              }}
            >
              Cancel
            </button>
            <button
              className="px-4 py-2 text-sm font-medium text-white bg-classic-blue rounded-md hover:bg-classic-blue-hover transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
              onClick={handleCopy}
              disabled={isLoading || !name.trim()}
            >
              {isLoading ? "Copying..." : "Copy Pipeline"}
            </button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
};

export default CopyPipelineButton;
