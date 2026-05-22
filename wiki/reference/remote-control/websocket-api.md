# WebSocket API (app side)

!!! todo "Stub - to be written"
    The message types the app sends/receives. New content from
    `code/remote-control-app/MyApp/api/api-messages.tsx` and `api-types.tsx`.
    Should agree with [the WebSocket API](../api.md).

## Message types

!!! todo
    Table from `api-messages.tsx` / `api-types.tsx`.

## Sequence

!!! todo
    A Mermaid sequence diagram of a typical control session.

    ```mermaid
    sequenceDiagram
        App->>ESP32: connect (ws://robot:81)
        ESP32-->>App: hello / state
        App->>ESP32: command
        ESP32-->>App: ack
    ```
    (placeholder - replace with the real protocol during authoring)
