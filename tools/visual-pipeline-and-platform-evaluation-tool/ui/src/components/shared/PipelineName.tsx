import { usePipelineName } from "@/hooks/usePipelines";

interface PipelineNameProps {
  pipelineId: string;
}

export const PipelineName = ({ pipelineId }: PipelineNameProps) => {
  let name = usePipelineName(pipelineId);

  // TODO: this need to be removed once the pipeline names are updated on the backend
  if (
    name ===
    "Smart Network Video Recorder (NVR) Proxy Pipeline - Analytics Branch"
  ) {
    name = "Smart NVR - Analytics";
  }

  if (
    name ===
    "Smart Network Video Recorder (NVR) Proxy Pipeline - Media Only Branch"
  ) {
    name = "Smart NVR - Media Only";
  }

  return <>{name}</>;
};
