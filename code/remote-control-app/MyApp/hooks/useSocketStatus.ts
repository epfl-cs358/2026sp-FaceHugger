import { useEffect, useState } from "react";
import { isConnected } from "../services/socket";

export const useSocketStatus = (intervalMs = 500) => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const poll = setInterval(() => {
      setConnected(isConnected());
    }, intervalMs);

    return () => clearInterval(poll);
  }, []);

  return connected;
};