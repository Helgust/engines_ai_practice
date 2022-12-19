#include "raylib.h"
#include <functional>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <thread>
#include "math.h"
//#include "tree.h"
#include "dungeonGen.h"
#include "dungeonUtils.h"

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static void draw_nav_grid(const char *input, size_t width, size_t height)
{
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
    {
      char symb = input[coord_to_idx(x, y, width)];
      Color color = GetColor(symb == ' ' ? 0xeeeeeeff : symb == 'o' ? 0x7777ffff : 0x222222ff);
      DrawPixel(int(x), int(y), color);
    }
}

static void draw_path(std::vector<Position> path, Color draw_color)
{
  for (const Position &p : path)
    DrawPixel(p.x, p.y, draw_color);
}

static void draw_multiple_path(std::vector<std::vector<Position>> paths, Color draw_color)
{
  for (const std::vector<Position> &p : paths)
  {
    draw_path(p, draw_color);
    draw_color.r -=5U;
    draw_color.g -=10U;
    draw_color.b += 8U;
  }
    
}

static std::vector<Position> reconstruct_path(std::vector<Position> prev, Position to, size_t width)
{
  Position curPos = to;
  std::vector<Position> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != Position{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<Position> find_path_a_star(const char *input, size_t width, size_t height, Position from, Position to)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();
  size_t inpSize = width * height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<Position> prev(inpSize, {-1,-1});

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };
  auto getF = [&](Position p) -> float { return f[coord_to_idx(p.x, p.y, width)]; };

  auto heuristic = [](Position lhs, Position rhs) -> float
  {
    return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
  };

  g[coord_to_idx(from.x, from.y, width)] = 0;
  f[coord_to_idx(from.x, from.y, width)] = heuristic(from, to);

  std::vector<Position> openList = {from};
  std::vector<Position> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, width);
    Position curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, width);
    DrawPixel(curPos.x, curPos.y, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float weight = input[idx] == 'o' ? 10.f : 1.f;
      float gScore = getG(curPos) + 1.f * weight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<Position>();
}

static std::vector<Position> find_path_sma_star(const char *input, size_t width, size_t height, Position from, Position to, int limit)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();
  size_t inpSize = width * height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<Position> prev(inpSize, {-1,-1});

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };
  auto getF = [&](Position p) -> float { return f[coord_to_idx(p.x, p.y, width)]; };

  auto heuristic = [](Position lhs, Position rhs) -> float
  {
    return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
  };

  g[coord_to_idx(from.x, from.y, width)] = 0;
  f[coord_to_idx(from.x, from.y, width)] = heuristic(from, to);

  int dist = ((int)sqrt(dist_sq(from, to)))*2;
  std::vector<Position> openList(dist);
  openList[0] = from;
  std::vector<Position> closedList(dist);

  while (!openList.empty())
  {
    int index = 0;
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, width);
    Position curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, width);
    DrawPixel(curPos.x, curPos.y, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
    closedList.emplace(closedList.begin() + index, curPos);
    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float weight = input[idx] == 'o' ? 10.f : 1.f;
      float gScore = getG(curPos) + 1.f * weight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
      
      if ((openList.size() + closedList.size()) > limit)
      {
        size_t biggestIdx = 0;
        size_t idx = openList.size() - 1;
        float biggestScore = getF(openList[idx]);
        for (size_t i = 1; i < openList.size(); ++i)
        {
          float score = getF(openList[i]);
          if (score > biggestScore)
          {
            biggestIdx = i;
            biggestScore = score;
          }
        }
        openList.erase(openList.begin() + biggestIdx);
      }
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<Position>();
}

static std::vector<std::vector<Position>> find_path_ara_star(const char *input, size_t width, size_t height, Position from, Position to)
{
  std::vector<std::vector<Position>> result;
  
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return result;
  size_t inpSize = width * height;
 
  std::vector<float> g(inpSize, std::numeric_limits<float>::max());

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };

  auto heuristic = [](Position lhs, Position rhs) -> float
  {
    return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
  };

  float epsilon = 15.0f;

  auto fval = [&](auto point) {
    return getG(point) + epsilon * heuristic(point,to);
  };

  auto getScore = [&](auto point) {
    return getG(point) + heuristic(point,to);
  };
  std::vector<Position> openList = {from};
  std::vector<Position> closedList;
  std::vector<Position> incons;

  g[coord_to_idx(from.x, from.y, width)] = 0;

  auto FindPath = [&]
  {
    std::vector<Position> prev(inpSize, {-1, -1});
    while(!openList.empty())
    {
      size_t bestIdx = 0;
      float minVal = fval(openList[0]);
      for (size_t i = 1; i < openList.size(); ++i)
      {
        float score = fval(openList[i]);
        if (score < minVal)
        {
          bestIdx = i;
          minVal = score;
        }
      }
      if (openList[bestIdx] == to)
        return reconstruct_path(prev, to, width);
      Position curPos = openList[bestIdx];
      openList.erase(openList.begin() + bestIdx);
      if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
        continue;
      size_t idx = coord_to_idx(curPos.x, curPos.y, width);
      DrawPixel(curPos.x, curPos.y, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
      closedList.emplace_back(curPos);

    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float weight = input[idx] == 'o' ? 10.f : 1.f;
      auto dist = heuristic(p, curPos);
      auto gScore = getG(curPos) + dist;
      if (getG(p) > gScore)
      {
        g[coord_to_idx(p.x, p.y, width)] = gScore;
        prev[idx] = curPos;
        bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
        if (!found)
          openList.emplace_back(p);
        else
          incons.emplace_back(p);
      }
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
    };
  };
  result.push_back(FindPath());
  auto get_new_epsilon = [&]{
    if (incons.empty()) {
      size_t bestIdx = 0;
      float minVal = getScore(openList[0]);
      for (size_t i = 1; i < openList.size(); ++i)
      {
        float score = getScore(openList[i]);
        if (score < minVal)
        {
          bestIdx = i;
          minVal = score;
        }
      }
      return getG(to) / minVal;
    }

    size_t bestIdx = 0;
    float minValOpen = getScore(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getScore(openList[i]);
      if (score < minValOpen)
      {
        bestIdx = i;
        minValOpen = score;
      }
    }
    float minValIncons = getScore(incons[0]);
    for (size_t i = 1; i < incons.size(); ++i)
    {
      float score = getScore(incons[i]);
      if (score < minValIncons)
      {
        bestIdx = i;
        minValIncons = score;
      }
    }
    return getG(to) / std::min(minValIncons, minValOpen);
  };
  float new_eps = std::min(epsilon, get_new_epsilon());
  
  while (new_eps > 1.0f) {
    epsilon *= 0.8f;
    for (int i = 0; i < incons.size(); ++i)
    {
      bool found = std::find(openList.begin(), openList.end(), incons[i]) != openList.end();
      if (!found)
        openList.emplace_back(incons[i]);
    }
    incons.clear();
    closedList.clear();
    result.push_back(FindPath());
    new_eps = std::min(epsilon, get_new_epsilon());
  }
  // empty paths
  return result;
}

void draw_nav_data(const char *input, size_t width, size_t height, Position from, Position to, 
  bool ara, int &index)
{
  draw_nav_grid(input, width, height);
  if (!ara)
  {
    std::vector<Position> path = find_path_a_star(input, width, height, from, to);
    std::vector<Position> path_sma = find_path_sma_star(input, width, height, from, to, 1500);
    draw_path(path, GetColor(0xFF0000FF));
    draw_path(path_sma, GetColor(0x00FF00FF));
  }
  else
  {
    std::vector<std::vector<Position>> paths = find_path_ara_star(input, width, height, from, to);
    draw_multiple_path(paths, GetColor(0xFFFF00FF));
  }
}

int main(int /*argc*/, const char ** /*argv*/)
{
  int width = 1920;
  int height = 1080;
  InitWindow(width, height, "w3 AI MIPT");

  const int scrWidth = GetMonitorWidth(0);
  const int scrHeight = GetMonitorHeight(0);
  if (scrWidth < width || scrHeight < height)
  {
    width = std::min(scrWidth, width);
    height = std::min(scrHeight - 150, height);
    SetWindowSize(width, height);
  }

  constexpr size_t dungWidth = 100;
  constexpr size_t dungHeight = 100;
  char *navGrid = new char[dungWidth * dungHeight];
  gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
  spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);

  Position from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
  Position to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);

  Camera2D camera = { {0, 0}, {0, 0}, 0.f, 1.f };
  //camera.offset = Vector2{ width * 0.5f, height * 0.5f };
  camera.zoom = float(height) / float(dungHeight);
  bool ara = true;
  int index = 0;
  SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
  while (!WindowShouldClose())
  {
    // pick pos
    Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
    Position p{int(mousePosition.x), int(mousePosition.y)};
    if (IsMouseButtonPressed(2) || IsKeyPressed(KEY_Q))
    {
      size_t idx = coord_to_idx(p.x, p.y, dungWidth);
      if (idx < dungWidth * dungHeight)
        navGrid[idx] = navGrid[idx] == ' ' ? '#' : navGrid[idx] == '#' ? 'o' : ' ';
    }
    else if (IsMouseButtonPressed(0))
    {
      Position &target = from;
      target = p;
    }
    else if (IsMouseButtonPressed(1))
    {
      Position &target = to;
      target = p;
    }
    else if (IsKeyPressed(KEY_LEFT_CONTROL))
    {
      ara = !ara;
    }
    else if (IsKeyPressed(KEY_F))
    {
      index += 1;
    }
    if (IsKeyPressed(KEY_SPACE))
    {
      gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
      spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
      from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
      to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
    }
    BeginDrawing();
      ClearBackground(BLACK);
      BeginMode2D(camera);
        draw_nav_data(navGrid, dungWidth, dungHeight, from, to, ara, index);
      EndMode2D();
    EndDrawing();
  }
  CloseWindow();
  return 0;
}
