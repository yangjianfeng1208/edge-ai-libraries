import { Square } from "lucide-react";

type StopPipelineButtonProps = {
  isStopping: boolean;
  onStopPipeline: () => void;
};

const StopPipelineButton = ({
  isStopping,
  onStopPipeline,
}: StopPipelineButtonProps) => (
  <button
    onClick={onStopPipeline}
    disabled={isStopping}
    className="bg-red-600 hover:bg-red-700 disabled:bg-gray-400 text-white p-2 rounded-lg shadow-lg transition-colors"
    title="Stop Pipeline"
  >
    <Square className="w-5 h-5" />
  </button>
);

export default StopPipelineButton;
