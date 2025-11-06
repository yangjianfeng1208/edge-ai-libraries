import { useWebSocketConnection } from "@/hooks/useWebSocketConnection.ts";
import type { ReactNode } from "react";

interface MetricsProviderProps {
  children: ReactNode;
}

/**
 * Provider component that manages the global WebSocket connection for metrics.
 * Should be placed high in the component tree to ensure the connection is available
 * throughout the application lifecycle.
 */
export const MetricsProvider = ({ children }: MetricsProviderProps) => {
  useWebSocketConnection();

  return <>{children}</>;
};
