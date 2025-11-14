import { useGetPipelinesQuery } from "@/api/api.generated.ts";
import { Link } from "react-router";
import {
  Card,
  CardHeader,
  CardTitle,
  CardDescription,
  CardFooter,
} from "@/components/ui/card";

const Home = () => {
  const { data: pipelines, isSuccess } = useGetPipelinesQuery();

  if (isSuccess && pipelines) {
    return (
      <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3 p-4">
        {pipelines.map((pipeline) => {
          const id = `${pipeline.name}-${pipeline.version}`;
          return (
            <Card key={id}>
              <CardHeader>
                <CardTitle>{pipeline.version}</CardTitle>
                <CardDescription>{pipeline.description}</CardDescription>
              </CardHeader>
              <CardFooter>
                <Link
                  to={`/pipelines/${pipeline.version}`}
                  className="bg-blue-500 text-white px-4 py-2 rounded hover:bg-blue-600 transition-colors"
                >
                  Open in Builder
                </Link>
              </CardFooter>
            </Card>
          );
        })}
      </div>
    );
  }

  return <div>Loading predefined pipelines...</div>;
};

export default Home;
