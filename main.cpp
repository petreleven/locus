#include "Core.h"
#include "LocusMathFunctions.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <raylib.h>
#include <vector>
int main() {

  std::vector<locus::Point> setOfPoints;
  for (size_t i = 0; i < 60; i++) {
    locus::Point p;
    locus::real x = 100 + rand() % 700;
    locus::real y = 50 + rand() % 300;
    p.x = x;
    p.y = y;
    p.z = 0.f;
    setOfPoints.push_back(p);
  }
  uint32_t bestIndex = locusMath::LMathFunctions::pointFarthestFromEdge(
      setOfPoints[2], setOfPoints[4], setOfPoints);

  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 800;
  const int screenHeight = 450;

  InitWindow(screenWidth, screenHeight, "test");

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    // TODO: Update your variables here
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(Color{26, 26, 29});
    for (size_t i = 0; i < setOfPoints.size(); i++) {
      DrawCircleV({setOfPoints[i].x, setOfPoints[i].y}, 10,
                  Color{166, 77, 121, 255});
    }

    DrawLineV({setOfPoints[2].x, setOfPoints[2].y},
              {setOfPoints[4].x, setOfPoints[4].y}, WHITE);
    DrawCircleV({setOfPoints[bestIndex].x, setOfPoints[bestIndex].y}, 5,
        GREEN);
    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
}
