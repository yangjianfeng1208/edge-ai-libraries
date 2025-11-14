import { apiSlice as api } from "./apiSlice";
export const addTagTypes = [
  "pipelines",
  "devices",
  "models",
  "videos",
  "convert",
] as const;
const injectedRtkApi = api
  .enhanceEndpoints({
    addTagTypes,
  })
  .injectEndpoints({
    endpoints: (build) => ({
      getPipelines: build.query<GetPipelinesApiResponse, GetPipelinesApiArg>({
        query: () => ({ url: `/pipelines` }),
        providesTags: ["pipelines"],
      }),
      createPipeline: build.mutation<
        CreatePipelineApiResponse,
        CreatePipelineApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines`,
          method: "POST",
          body: queryArg.pipelineDefinition,
        }),
        invalidatesTags: ["pipelines"],
      }),
      validatePipeline: build.mutation<
        ValidatePipelineApiResponse,
        ValidatePipelineApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/validate`,
          method: "POST",
          body: queryArg.pipelineValidation,
        }),
        invalidatesTags: ["pipelines"],
      }),
      getPipelineStatuses: build.query<
        GetPipelineStatusesApiResponse,
        GetPipelineStatusesApiArg
      >({
        query: () => ({ url: `/pipelines/status` }),
        providesTags: ["pipelines"],
      }),
      getPipelineInstanceSummary: build.query<
        GetPipelineInstanceSummaryApiResponse,
        GetPipelineInstanceSummaryApiArg
      >({
        query: (queryArg) => ({ url: `/pipelines/${queryArg.instanceId}` }),
        providesTags: ["pipelines"],
      }),
      stopPipelineInstance: build.mutation<
        StopPipelineInstanceApiResponse,
        StopPipelineInstanceApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.instanceId}`,
          method: "DELETE",
        }),
        invalidatesTags: ["pipelines"],
      }),
      getPipelineInstanceStatus: build.query<
        GetPipelineInstanceStatusApiResponse,
        GetPipelineInstanceStatusApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.instanceId}/status`,
        }),
        providesTags: ["pipelines"],
      }),
      getPipeline: build.query<GetPipelineApiResponse, GetPipelineApiArg>({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.name}/${queryArg.version}`,
        }),
        providesTags: ["pipelines"],
      }),
      runPipeline: build.mutation<RunPipelineApiResponse, RunPipelineApiArg>({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.name}/${queryArg.version}`,
          method: "POST",
          body: queryArg.pipelineRequestRunInput,
        }),
        invalidatesTags: ["pipelines"],
      }),
      deletePipeline: build.mutation<
        DeletePipelineApiResponse,
        DeletePipelineApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.name}/${queryArg.version}`,
          method: "DELETE",
        }),
        invalidatesTags: ["pipelines"],
      }),
      benchmarkPipeline: build.mutation<
        BenchmarkPipelineApiResponse,
        BenchmarkPipelineApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.name}/${queryArg.version}/benchmark`,
          method: "POST",
          body: queryArg.pipelineRequestBenchmarkInput,
        }),
        invalidatesTags: ["pipelines"],
      }),
      optimizePipeline: build.mutation<
        OptimizePipelineApiResponse,
        OptimizePipelineApiArg
      >({
        query: (queryArg) => ({
          url: `/pipelines/${queryArg.name}/${queryArg.version}/optimize`,
          method: "POST",
          body: queryArg.pipelineRequestOptimize,
        }),
        invalidatesTags: ["pipelines"],
      }),
      getDevices: build.query<GetDevicesApiResponse, GetDevicesApiArg>({
        query: () => ({ url: `/devices` }),
        providesTags: ["devices"],
      }),
      getModels: build.query<GetModelsApiResponse, GetModelsApiArg>({
        query: () => ({ url: `/models` }),
        providesTags: ["models"],
      }),
      getVideos: build.query<GetVideosApiResponse, GetVideosApiArg>({
        query: () => ({ url: `/videos` }),
        providesTags: ["videos"],
      }),
      toGraph: build.mutation<ToGraphApiResponse, ToGraphApiArg>({
        query: (queryArg) => ({
          url: `/convert/to-graph`,
          method: "POST",
          body: queryArg.pipelineDescription,
        }),
        invalidatesTags: ["convert"],
      }),
      toDescription: build.mutation<
        ToDescriptionApiResponse,
        ToDescriptionApiArg
      >({
        query: (queryArg) => ({
          url: `/convert/to-description`,
          method: "POST",
          body: queryArg.pipelineGraph,
        }),
        invalidatesTags: ["convert"],
      }),
    }),
    overrideExisting: false,
  });
export { injectedRtkApi as api };
export type GetPipelinesApiResponse =
  /** status 200 Successful Response */ Pipeline[];
export type GetPipelinesApiArg = void;
export type CreatePipelineApiResponse =
  /** status 201 Pipeline created */ MessageResponse;
export type CreatePipelineApiArg = {
  pipelineDefinition: PipelineDefinition;
};
export type ValidatePipelineApiResponse =
  /** status 200 Pipeline is valid */ MessageResponse;
export type ValidatePipelineApiArg = {
  pipelineValidation: PipelineValidation;
};
export type GetPipelineStatusesApiResponse =
  /** status 200 Successful Response */ PipelineInstanceStatus[];
export type GetPipelineStatusesApiArg = void;
export type GetPipelineInstanceSummaryApiResponse =
  /** status 200 Successful Response */ PipelineInstanceSummary;
export type GetPipelineInstanceSummaryApiArg = {
  instanceId: string;
};
export type StopPipelineInstanceApiResponse =
  /** status 200 Successful Response */ MessageResponse;
export type StopPipelineInstanceApiArg = {
  instanceId: string;
};
export type GetPipelineInstanceStatusApiResponse =
  /** status 200 Successful Response */ PipelineInstanceStatus;
export type GetPipelineInstanceStatusApiArg = {
  instanceId: string;
};
export type GetPipelineApiResponse =
  /** status 200 Successful Response */ Pipeline;
export type GetPipelineApiArg = {
  name: string;
  version: string;
};
export type RunPipelineApiResponse =
  /** status 202 Successful Response */ PipelineInstanceResponse;
export type RunPipelineApiArg = {
  name: string;
  version: string;
  pipelineRequestRunInput: PipelineRequestRun2;
};
export type DeletePipelineApiResponse =
  /** status 200 Pipeline deleted */ MessageResponse;
export type DeletePipelineApiArg = {
  name: string;
  version: string;
};
export type BenchmarkPipelineApiResponse =
  /** status 202 Successful Response */ PipelineInstanceResponse;
export type BenchmarkPipelineApiArg = {
  name: string;
  version: string;
  pipelineRequestBenchmarkInput: PipelineRequestBenchmark2;
};
export type OptimizePipelineApiResponse =
  /** status 200 Successful Response */ any;
export type OptimizePipelineApiArg = {
  name: string;
  version: string;
  pipelineRequestOptimize: PipelineRequestOptimize;
};
export type GetDevicesApiResponse =
  /** status 200 Successful Response */ Device[];
export type GetDevicesApiArg = void;
export type GetModelsApiResponse =
  /** status 200 Successful Response */ Model[];
export type GetModelsApiArg = void;
export type GetVideosApiResponse =
  /** status 200 Successful Response */ Video[];
export type GetVideosApiArg = void;
export type ToGraphApiResponse =
  /** status 200 Conversion successful */ PipelineGraph;
export type ToGraphApiArg = {
  pipelineDescription: PipelineDescription;
};
export type ToDescriptionApiResponse =
  /** status 200 Conversion successful */ PipelineDescription;
export type ToDescriptionApiArg = {
  pipelineGraph: PipelineGraph;
};
export type PipelineType = "GStreamer" | "FFmpeg";
export type Node = {
  id: string;
  type: string;
  data: {
    [key: string]: string;
  };
};
export type Edge = {
  id: string;
  source: string;
  target: string;
};
export type PipelineGraph = {
  nodes: Node[];
  edges: Edge[];
};
export type PipelineParameters = {
  default: {
    [key: string]: any;
  } | null;
};
export type Pipeline = {
  name: string;
  version: string;
  description: string;
  type: PipelineType;
  pipeline_graph: PipelineGraph;
  parameters: PipelineParameters | null;
};
export type MessageResponse = {
  message: string;
};
export type ValidationError = {
  loc: (string | number)[];
  msg: string;
  type: string;
};
export type HttpValidationError = {
  detail?: ValidationError[];
};
export type PipelineDefinition = {
  name: string;
  version: string;
  description: string;
  type: PipelineType;
  pipeline_description: string;
  parameters: PipelineParameters | null;
};
export type PipelineValidation = {
  type: PipelineType;
  pipeline_description: string;
  parameters: PipelineParameters | null;
};
export type PipelineInstanceState =
  | "RUNNING"
  | "COMPLETED"
  | "ERROR"
  | "ABORTED";
export type PipelineInstanceStatus = {
  id: string;
  start_time: number;
  elapsed_time: number;
  state: PipelineInstanceState;
  total_fps: number | null;
  per_stream_fps: number | null;
  ai_streams: number | null;
  non_ai_streams: number | null;
  error_message: string | null;
};
export type SourceType = "uri" | "gst";
export type Source = {
  type: SourceType;
  uri: string | null;
};
export type PipelineParametersRun = {
  inferencing_channels?: number;
  recording_channels?: number;
  pipeline_graph: PipelineGraph;
};
export type PipelineRequestRun = {
  async_?: boolean | null;
  source: Source;
  parameters: PipelineParametersRun;
  tags: {
    [key: string]: string;
  } | null;
};
export type PipelineParametersBenchmark = {
  fps_floor?: number;
  ai_stream_rate?: number;
  pipeline_graph: PipelineGraph;
};
export type PipelineRequestBenchmark = {
  async_?: boolean | null;
  source: Source;
  parameters: PipelineParametersBenchmark;
  tags: {
    [key: string]: string;
  } | null;
};
export type PipelineInstanceSummary = {
  id: string;
  request: PipelineRequestRun | PipelineRequestBenchmark;
  type: string;
};
export type PipelineInstanceResponse = {
  instance_id: string;
};
export type PipelineParametersRun2 = {
  inferencing_channels?: number;
  recording_channels?: number;
  pipeline_graph: PipelineGraph;
};
export type PipelineRequestRun2 = {
  async_?: boolean | null;
  source: Source;
  parameters: PipelineParametersRun2;
  tags: {
    [key: string]: string;
  } | null;
};
export type PipelineParametersBenchmark2 = {
  fps_floor?: number;
  ai_stream_rate?: number;
  pipeline_graph: PipelineGraph;
};
export type PipelineRequestBenchmark2 = {
  async_?: boolean | null;
  source: Source;
  parameters: PipelineParametersBenchmark2;
  tags: {
    [key: string]: string;
  } | null;
};
export type PipelineRequestOptimize = {
  async_?: boolean | null;
  source: Source;
  parameters: {
    [key: string]: any;
  } | null;
  tags: {
    [key: string]: string;
  } | null;
};
export type DeviceType = "DISCRETE" | "INTEGRATED";
export type DeviceFamily = "CPU" | "GPU" | "NPU";
export type Device = {
  device_name: string;
  full_device_name: string;
  device_type: DeviceType;
  device_family: DeviceFamily;
  gpu_id: number | null;
};
export type ModelCategory = "classification" | "detection";
export type Model = {
  name: string;
  display_name: string;
  category: ModelCategory | null;
  precision: string | null;
};
export type Video = {
  filename: string;
  width: number;
  height: number;
  fps: number;
  frame_count: number;
  codec: string;
  duration: number;
};
export type PipelineDescription = {
  pipeline_description: string;
};
export const {
  useGetPipelinesQuery,
  useLazyGetPipelinesQuery,
  useCreatePipelineMutation,
  useValidatePipelineMutation,
  useGetPipelineStatusesQuery,
  useLazyGetPipelineStatusesQuery,
  useGetPipelineInstanceSummaryQuery,
  useLazyGetPipelineInstanceSummaryQuery,
  useStopPipelineInstanceMutation,
  useGetPipelineInstanceStatusQuery,
  useLazyGetPipelineInstanceStatusQuery,
  useGetPipelineQuery,
  useLazyGetPipelineQuery,
  useRunPipelineMutation,
  useDeletePipelineMutation,
  useBenchmarkPipelineMutation,
  useOptimizePipelineMutation,
  useGetDevicesQuery,
  useLazyGetDevicesQuery,
  useGetModelsQuery,
  useLazyGetModelsQuery,
  useGetVideosQuery,
  useLazyGetVideosQuery,
  useToGraphMutation,
  useToDescriptionMutation,
} = injectedRtkApi;
