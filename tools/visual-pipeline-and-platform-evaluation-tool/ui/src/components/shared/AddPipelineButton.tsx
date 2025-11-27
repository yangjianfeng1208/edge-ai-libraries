import { useState } from "react";
import { Plus } from "lucide-react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { useCreatePipelineMutation } from "@/api/api.generated";
import { toast } from "sonner";

const AddPipelineButton = () => {
  const [open, setOpen] = useState(false);
  const [name, setName] = useState("");
  const [description, setDescription] = useState("");
  const [pipelineDescription, setPipelineDescription] = useState("");
  const [createPipeline, { isLoading }] = useCreatePipelineMutation();

  const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      const content = e.target?.result as string;
      setPipelineDescription(content);
    };
    reader.readAsText(file);
  };

  const handleAdd = async () => {
    if (!name.trim() || !pipelineDescription.trim()) {
      toast.error("Name and pipeline description are required");
      return;
    }

    try {
      await createPipeline({
        pipelineDefinition: {
          name: name.trim(),
          description: description.trim(),
          source: "USER_CREATED",
          type: "GStreamer",
          pipeline_description: pipelineDescription,
          parameters: {
            default: {
              additionalProp1: {},
            },
          },
        },
      }).unwrap();

      toast.success("Pipeline created successfully");
      setOpen(false);
      setName("");
      setDescription("");
      setPipelineDescription("");
    } catch (error) {
      toast.error("Failed to create pipeline", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
      console.error("Failed to create pipeline:", error);
    }
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <button
          className="w-full h-full min-h-[200px] border-2 border-dashed border-gray-300 dark:border-gray-700 rounded-lg hover:border-blue-500 hover:bg-blue-50 dark:hover:bg-blue-950/20 transition-all flex flex-col items-center justify-center gap-3 text-gray-500 dark:text-gray-400 hover:text-blue-600 dark:hover:text-blue-400"
          title="Add Pipeline"
        >
          <Plus className="w-12 h-12" />
          <span className="text-lg font-medium">Add New Pipeline</span>
        </button>
      </DialogTrigger>
      <DialogContent className="!max-w-6xl">
        <DialogHeader>
          <DialogTitle>Add New Pipeline</DialogTitle>
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
            <input
              id="description"
              type="text"
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              placeholder="Enter pipeline description..."
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>

          <div>
            <label
              htmlFor="file-upload"
              className="block text-sm font-medium mb-2"
            >
              Upload file with Pipeline Description (.txt)
            </label>
            <input
              id="file-upload"
              type="file"
              accept=".txt"
              onChange={handleFileUpload}
              className="block w-full text-sm text-gray-500 file:mr-4 file:py-2 file:px-4 file:rounded file:border-0 file:text-sm file:font-semibold file:bg-primary file:text-primary-foreground hover:file:bg-primary/90"
            />
          </div>

          <div>
            <label
              htmlFor="pipeline-description"
              className="block text-sm font-medium mb-2"
            >
              Pipeline Description
            </label>
            <textarea
              id="pipeline-description"
              value={pipelineDescription}
              onChange={(e) => setPipelineDescription(e.target.value)}
              placeholder="Paste or upload your pipeline description here..."
              className="w-full h-64 p-3 border rounded-md resize-none font-mono text-sm"
            />
          </div>

          <div className="flex justify-end gap-2">
            <button
              className="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 transition-colors"
              onClick={() => {
                setOpen(false);
                setName("");
                setDescription("");
                setPipelineDescription("");
              }}
            >
              Cancel
            </button>
            <button
              className="px-4 py-2 text-sm font-medium text-white bg-blue-600 rounded-md hover:bg-blue-700 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
              onClick={handleAdd}
              disabled={
                isLoading || !name.trim() || !pipelineDescription.trim()
              }
            >
              {isLoading ? "Creating..." : "Add"}
            </button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
};

export default AddPipelineButton;
