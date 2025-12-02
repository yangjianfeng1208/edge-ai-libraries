import { configureStore } from "@reduxjs/toolkit";
import { api } from "@/api/api.generated.ts";
import metricsReducer from "./reducers/metrics.ts";
import pipelinesReducer from "./reducers/pipelines.ts";

export const store = configureStore({
  reducer: {
    [api.reducerPath]: api.reducer,
    metrics: metricsReducer,
    pipelines: pipelinesReducer,
  },
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware().concat(api.middleware),
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;
