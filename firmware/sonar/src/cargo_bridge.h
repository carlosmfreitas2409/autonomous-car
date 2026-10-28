#pragma once

class MCP2515;

namespace cargo_bridge {
void init(MCP2515& mcp);

void update();

}
