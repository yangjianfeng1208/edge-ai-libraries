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
import { useAppSelector } from "@/store/hooks";
import { selectPipelines } from "@/store/reducers/pipelines";

const Home = () => {
  const pipelines = useAppSelector(selectPipelines);

  if (pipelines.length > 0) {
    return (
      <div className="flex h-full">
        <div className="flex-1 overflow-auto">
          <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3 p-4">
            <AddPipelineButton />
            {pipelines.map((pipeline) => (
              <Card key={pipeline.id}>
                <CardHeader>
                  <CardTitle>{pipeline.name}</CardTitle>
                  <CardDescription>{pipeline.description}</CardDescription>
                </CardHeader>
                <CardFooter>
                  <Link
                    to={`/pipelines/${pipeline.id}`}
                    className="bg-blue-500 text-white px-4 py-2 rounded hover:bg-blue-600 transition-colors"
                  >
                    Open in Builder
                  </Link>
                </CardFooter>
              </Card>
            ))}
          </div>
        </div>
        <div className="w-[25%] border-l p-4 flex flex-col gap-4">
          <h1 className="font-medium text-2xl">Resource utilization</h1>
          <CpuUsageProgress />
          <GpuUsageProgress />
        </div>
      </div>
    );
  }

  return <div>Loading predefined pipelines...</div>;
};

export default Home;
