import { Play } from "lucide-react";

type RunPipelineButtonProps = {
  isRunning: boolean;
  onRunPipeline: () => void;
};

const RunPipelineButton = ({
  isRunning,
  onRunPipeline,
}: RunPipelineButtonProps) => (
  <button
    onClick={onRunPipeline}
    disabled={isRunning}
    className="bg-green-600 hover:bg-green-700 disabled:bg-gray-400 text-white p-2 rounded-lg shadow-lg transition-colors"
    title="Run Pipeline"
  >
    <Play className="w-5 h-5" />
  </button>
);

export default RunPipelineButton;
