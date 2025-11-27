import { useEffect, useState } from "react";
import {
  useGetPerformanceJobStatusQuery,
  useGetPipelinesQuery,
  useRunPerformanceTestMutation,
} from "@/api/api.generated";
import {
  PipelinesDataTable,
  type PipelineWithStreams,
} from "@/components/shared/PipelinesDataTable";
import { TestProgressIndicator } from "@/components/shared/TestProgressIndicator.tsx";

const PerformanceTests = () => {
  const { data: pipelines, isLoading, error } = useGetPipelinesQuery();
  const [runPerformanceTest, { isLoading: isRunning }] =
    useRunPerformanceTestMutation();
  const [selectedPipelines, setSelectedPipelines] = useState<
    PipelineWithStreams[]
  >([]);
  const [jobId, setJobId] = useState<string | null>(null);
  const [testResult, setTestResult] = useState<{
    total_fps: number | null;
    per_stream_fps: number | null;
  } | null>(null);

  const { data: jobStatus } = useGetPerformanceJobStatusQuery(
    { jobId: jobId! },
    {
      skip: !jobId,
      pollingInterval: 1000, // Poll every second
    },
  );

  useEffect(() => {
    if (jobStatus?.state === "COMPLETED") {
      setTestResult({
        total_fps: jobStatus.total_fps,
        per_stream_fps: jobStatus.per_stream_fps,
      });
      setJobId(null);
    } else if (jobStatus?.state === "ERROR" || jobStatus?.state === "ABORTED") {
      console.error("Test failed:", jobStatus.error_message);
      setJobId(null);
    }
  }, [jobStatus]);

  const handleRunTest = async () => {
    if (selectedPipelines.length === 0) return;

    setTestResult(null);
    try {
      const result = await runPerformanceTest({
        performanceTestSpec: {
          pipeline_performance_specs: selectedPipelines.map((pipeline) => ({
            id: pipeline.id,
            streams: pipeline.streams,
          })),
        },
      }).unwrap();
      setJobId(result.job_id);
    } catch (err) {
      console.error("Failed to run performance test:", err);
    }
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-full">
        <p>Loading pipelines...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex items-center justify-center h-full">
        <p className="text-red-500">Error loading pipelines</p>
      </div>
    );
  }

  return (
    <div className="container mx-auto py-10">
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Performance Tests</h1>
        <p className="text-muted-foreground mt-2">
          Select pipelines to run performance tests on
        </p>
      </div>

      <PipelinesDataTable
        data={pipelines ?? []}
        onSelectionChange={setSelectedPipelines}
      />

      <div className="my-4">
        <button
          onClick={handleRunTest}
          disabled={isRunning || selectedPipelines.length === 0 || !!jobId}
          className="px-4 py-2 bg-primary text-primary-foreground rounded-md hover:bg-primary/90 disabled:opacity-50 disabled:cursor-not-allowed"
        >
          {jobId ? "Running..." : isRunning ? "Starting..." : "Run test"}
        </button>
      </div>

      {jobId && jobStatus && (
        <div className="mb-4 p-3 bg-blue-50 dark:bg-blue-950 border border-blue-200 dark:border-blue-800 rounded-md">
          <p className="text-sm font-medium text-blue-900 dark:text-blue-100">
            Test Status: {jobStatus.state}
          </p>
          {jobStatus.state === "RUNNING" && (
            <div className="mt-2">
              <div className="animate-pulse flex items-center gap-2">
                <div className="h-2 w-2 bg-blue-500 rounded-full"></div>
                <span className="text-xs text-blue-700 dark:text-blue-300">
                  Running performance test...
                </span>
              </div>
              <TestProgressIndicator />
            </div>
          )}
          {jobStatus.state === "ERROR" && (
            <div className="mt-2">
              <div className="animate-pulse flex items-center gap-2">
                <div className="h-2 w-2 bg-blue-500 rounded-full"></div>
                <span className="text-xs text-blue-700 dark:text-blue-300">
                  {jobStatus.error_message}
                </span>
              </div>
            </div>
          )}
        </div>
      )}

      {testResult && (
        <div className="mb-4 p-3 bg-green-50 dark:bg-green-950 border border-green-200 dark:border-green-800 rounded-md">
          <p className="text-sm font-medium text-green-900 dark:text-green-100 mb-2">
            Test Completed Successfully
          </p>
          <div className="space-y-1 text-sm">
            <p className="text-green-800 dark:text-green-200">
              <span className="font-medium">Total FPS:</span>{" "}
              {testResult.total_fps?.toFixed(2) ?? "N/A"}
            </p>
            <p className="text-green-800 dark:text-green-200">
              <span className="font-medium">Per Stream FPS:</span>{" "}
              {testResult.per_stream_fps?.toFixed(2) ?? "N/A"}
            </p>
          </div>
        </div>
      )}
    </div>
  );
};

export default PerformanceTests;
