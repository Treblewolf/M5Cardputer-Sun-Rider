# ☀️ Sun Rider for M5Stack Cardputer ☀️

**A 2D physics-based side-scrolling driving game with procedurally generated terrain.**

* An alien has depleted his magic goo fuel, and now has to use petrol like a real man. <em>Unfortunately the fumes got the best of him and now he's on an endless voyage to get sober.</em>🌌
  
* Available in M5Burner and M5Launcher!
## Features
![sunrider_menu](https://github.com/user-attachments/assets/e329ece9-9d46-428a-bdf4-8a3ec3cb84cd)
![sunrider_game](https://github.com/user-attachments/assets/5f3f97ad-3f9b-45c8-b077-4f4bfdeab9d5) 
* ⚙️ **Custom Physics Engine:**
    * Simulates a chassis and two wheels connected by suspension.
    * Includes gravity, drive torque, leaning torque, suspension forces, friction, and air resistance.
    * Uses physics sub-stepping for improved stability.
    * Handles collisions between the vehicle, rider, and terrain.
      
* 🏞️ **Procedural Terrain:** 
    * Endless, varied landscape generated on-the-fly using line segments.
    * Checkpoints mark distance milestones.
      
* **Dynamic Backgrounds:**
    * Parallax scrolling Starfield, Nebula, and Meteor Shower effects.
    * Smooth transitions between background types based on distance traveled.
      
* 🎨 **Graphics & Rendering:** 
    * Utilizes the M5GFX library via the M5Cardputer library.
    * Renders to an off-screen buffer for smooth, flicker-free animation.
    * Rotating sprites for the vehicle and rider.
    * Particle system for wheel dirt effects.
      
* 🎮 **Gameplay & UI:**
    * Simple controls for acceleration and leaning.
    * Game over on crashing or falling.
    * Tracks total distance and saves the high score.
    * Configurable on-screen display for FPS and distance.
    * Main Menu and detailed Options Menu.
      
* 💾 **Options & Persistence:**
    * Adjustable settings: Brightness, Volume, Gravity, Torque, Info Corner, Smooth Camera, Terrain Color, Menu Background.
    * Settings and high score saved persistently using the Preferences library.
      
* 🔊 **Audio:**
    * Simple synthesized sound effects for engine, menus, game over, and high score.

## Gameplay & Controls ⌨️

* **Objective:** Drive as far as possible without crashing.
* **Controls (Default Keyboard):**
    * `Space`: Accelerate
    * `A`: Lean Left
    * `D`: Lean Right
    * **Menus:**
        * `Up Arrow`: Navigate Up
        * `Down Arrow`: Navigate Down
        * `Left Arrow`: Decrease Value (in Options)
        * `Right Arrow`: Increase Value (in Options)
        * `Space`: Select / Confirm
        * `Esc`: Back (from Options)

## 🛠️ Building & Installation

This project is designed for the M5Stack Cardputer.
1.  **Prerequisites:**
    * [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/) installed. 💻
    * M5Stack Board definitions installed for your IDE.
    * [M5Cardputer Library](https://github.com/m5stack/M5Cardputer) installed. (This library includes the necessary M5GFX dependency). 📚
2.  **Clone the Repository:**
    ```bash
    git clone https://github.com/Treblewolf/M5Cardputer-Sun-Rider.git
    cd M5Cardputer-Sun-Rider
    ```
3. ⚡ **Build & Upload:**
    * **Arduino IDE:**
    * Open the `.ino` file, select the correct M5Stack Cardputer board from the Tools menu, go to `Sketch` and `Export Compiled Binary`
    * Copy the compiled `.bin` file onto SD card and run through M5Launcher

## 💡 Technical Highlights
* **Vec2D Struct:** Custom 2D vector implementation for physics calculations.
* **State Machine:** Simple `enum GameState` manages game flow.
* **Scrolling Options Menu:** Handles more options than fit on the screen.
* **Smooth Camera:** Toggable camera smoothing for player following. 🎥
* **Color Fading/Blending:** Helper functions for visual effects.
* **Sprite Rendering:** Uses `pushImageRotateZoomWithAA` for anti-aliased rotated sprites.
* **Background Management:** Intelligent wrapping and resetting of background elements.

## 🙌Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/Treblewolf/M5Cardputer-Sun-Rider/issues).

## 📄 License

Distributed under the MIT License. See `LICENSE` file for more information.
