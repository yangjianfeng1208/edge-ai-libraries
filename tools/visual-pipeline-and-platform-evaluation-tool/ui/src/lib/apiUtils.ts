import type { MessageResponse } from "@/api/api.generated";

export type RTKQueryError = {
  status: number;
  data: MessageResponse;
};

export const isApiError = (error: unknown): error is RTKQueryError => {
  return (
    typeof error === "object" &&
    error !== null &&
    "status" in error &&
    "data" in error &&
    typeof (error as RTKQueryError).data === "object" &&
    (error as RTKQueryError).data !== null &&
    "message" in (error as RTKQueryError).data
  );
};
