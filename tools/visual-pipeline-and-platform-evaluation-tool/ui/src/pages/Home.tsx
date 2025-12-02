import { Link } from "react-router";
import {
  Card,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { CpuUsageProgress } from "@/features/metrics/CpuUsageProgress.tsx";
import { GpuUsageProgress } from "@/features/metrics/GpuUsageProgress.tsx";
import AddPipelineButton from "@/components/shared/AddPipelineButton.tsx";
import CopyPipelineButton from "@/components/shared/CopyPipelineButton.tsx";
import { useAppSelector } from "@/store/hooks";
import { selectPipelines } from "@/store/reducers/pipelines";
import { BookOpen, Sparkles, Code } from "lucide-react";

const Home = () => {
  const pipelines = useAppSelector(selectPipelines);

  const predefinedPipelines =
    pipelines?.filter((p) => p.source === "PREDEFINED") ?? [];
  const userDefinedPipelines =
    pipelines?.filter((p) => p.source === "USER_CREATED") ?? [];

  if (pipelines.length > 0) {
    return (
      <div className="flex h-full">
        <div className="flex-1 overflow-auto">
          <div className="p-4 space-y-8">
            <div>
              <h2 className="text-xl font-semibold mb-4">
                Predefined Pipelines
              </h2>
              <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
                {predefinedPipelines.map((pipeline) => (
                  <Card key={pipeline.id} className="flex flex-col">
                    <CardHeader className="flex-1">
                      <CardTitle>{pipeline.name}</CardTitle>
                      <CardDescription className="line-clamp-4 min-h-[4.5rem]">
                        {pipeline.description}
                      </CardDescription>
                    </CardHeader>
                    <CardFooter>
                      <CopyPipelineButton pipeline={pipeline}>
                        <button className="text-white bg-classic-blue hover:bg-classic-blue-hover px-4 py-2 transition-colors">
                          Copy pipeline
                        </button>
                      </CopyPipelineButton>
                    </CardFooter>
                  </Card>
                ))}
              </div>
            </div>

            <div>
              <h2 className="text-xl font-semibold mb-4">
                User Defined Pipelines
              </h2>
              <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
                <AddPipelineButton />
                {userDefinedPipelines.map((pipeline) => (
                  <Card key={pipeline.id} className="flex flex-col">
                    <CardHeader className="flex-1">
                      <CardTitle>{pipeline.name}</CardTitle>
                      <CardDescription className="line-clamp-4 min-h-[4.5rem]">
                        {pipeline.description}
                      </CardDescription>
                    </CardHeader>
                    <CardFooter>
                      <Link
                        to={`/pipelines/${pipeline.id}`}
                        className="text-white bg-classic-blue hover:bg-classic-blue-hover px-4 py-2 transition-colors"
                      >
                        Open in Builder
                      </Link>
                    </CardFooter>
                  </Card>
                ))}
              </div>
            </div>
          </div>
        </div>
        <div className="w-[25%] border-l p-4 flex flex-col gap-4 bg-carbon-tint-2">
          <h1 className="font-medium text-2xl">Resource utilization</h1>
          <CpuUsageProgress />
          <GpuUsageProgress />

          <h1 className="font-medium text-2xl mt-4">Learning and support</h1>

          <div className="flex gap-3">
            <BookOpen className="w-6 h-6 text-classic-blue shrink-0" />
            <a
              href="https://docs.openedgeplatform.intel.com/2025.2/edge-ai-libraries/visual-pipeline-and-platform-evaluation-tool/get-started.html"
              target="_blank"
              rel="noopener noreferrer"
              className="hover:text-classic-blue transition-colors"
            >
              <h3 className="font-semibold text-base mb-1">Getting Started</h3>
              <p className="text-sm text-muted-foreground">
                Learn the fundamentals to get the most out of the Visual
                Pipeline and Platform Evaluation Tool (ViPPET)
              </p>
            </a>
          </div>

          <div className="flex gap-3">
            <Sparkles className="w-6 h-6 text-classic-blue shrink-0" />
            <a
              href="https://docs.openedgeplatform.intel.com/2025.2/edge-ai-libraries/visual-pipeline-and-platform-evaluation-tool/release-notes.html"
              target="_blank"
              rel="noopener noreferrer"
              className="hover:text-classic-blue transition-colors"
            >
              <h3 className="font-semibold text-base mb-1">What's new?</h3>
              <p className="text-sm text-muted-foreground">
                Check out what's new in the latest ViPPET 2025.2 release
              </p>
            </a>
          </div>

          <div className="flex gap-3">
            <Code className="w-6 h-6 text-classic-blue shrink-0" />
            <a
              href="/api/v1/redoc"
              target="_blank"
              rel="noopener noreferrer"
              className="hover:text-classic-blue transition-colors"
            >
              <h3 className="font-semibold text-base mb-1">REST API</h3>
              <p className="text-sm text-muted-foreground">
                You can use ViPPET also through REST API - see OpenAPI
                specification
              </p>
            </a>
          </div>
        </div>
      </div>
    );
  }

  return <div>Loading pipelines...</div>;
};

export default Home;
