#pragma once

class MCP2515;

// Bridges the cargo node's serial link onto the CAN bus, both ways.
namespace cargo_bridge {

// Also sets the MCP2515's acceptance filters to the cargo command alone.
void init(MCP2515& mcp);

// Never blocks; call every loop.
void update();

}  // namespace cargo_bridge
