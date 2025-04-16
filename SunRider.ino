/*
 * Sun Rider v1.0 - For M5Cardputer
 * Made by Treblewolf
 * https://github.com/Treblewolf/M5Cardputer-Sun-Rider
 * ====================================
 *
 * A 2D physics-based side-scrolling driving game for the M5Cardputer. Inspired by such classics as Elastomania, Gravity Defied, Hill Climb Racing
 *      Lore (lol):
 * An alien has depleted his magic goo fuel, and now has to use petrol like a real man. Unfortunately the fumes got the best of him and now he's on an endless voyage to get sober.
 *
 * --- Key Features ---
 *
 * - Custom Physics Engine (veery wacky atm):
 * -    Simulates a chassis and two wheels connected by suspension.
 * -    Includes gravity, drive torque, leaning torque, suspension spring/damping forces.
 * -    Models friction, air resistance (linear and angular).
 * -    Uses physics sub-stepping (PHYSICS_SUBSTEPS) for improved stability at higher speeds.
 * -    Collision detection between wheels/chassis/rider and terrain.
 * - Procedural Terrain Generation:
 * -    Creates an endless, varied landscape using connected line segments.
 * -    Terrain is generated on-the-fly ahead of the player and discarded behind.
 * -    Checkpoints mark distance milestones.
 * - Dynamic Backgrounds with Parallax:
 * -    Starfield: Simple stars with varying parallax speeds.
 * -    Nebula: Multi-layered sine wave patterns creating a colorful nebula effect, also with parallax.
 * -    Meteor Shower: Falling meteors with parallax scrolling.
 * -    Smooth transitions between background types based on distance traveled (Starfield -> Nebula -> Meteor -> Starfield).
 * - Graphics & Rendering:
 * -    Utilizes the M5GFX/LovyanGFX library for display output.
 * -    Renders to an off-screen buffer (M5Canvas) for flicker-free drawing.
 * -    Player vehicle represented by sprites (UFO chassis, rider) with rotation.
 * -    Particle system for dirt kicked up by the wheels.
 * -    Customizable terrain surface color.
 * - Gameplay & UI:
 * -    Drive using keyboard controls (Accelerate - Space, Lean - Left/Right Arrows).
 * -    Game over occurs on crashing the chassis or rider's head, or falling off-screen.
 * -    Tracks total distance traveled and high score.
 * -    On-screen display for FPS and distance (position configurable via options).
 * -    Main Menu and expansive Options Menu with wrapping.
 * - Options & Persistence:
 * -    Adjustable settings: Screen Brightness, Sound Volume, Gravity, Drive Torque, Info Corner Position, Smooth Camera, Terrain Color, Menu Background Type.
 * -    Settings and high score are saved persistently to NVS using the Preferences library.
 * - Audio:
 * -    Simple synthesized sound effects for engine noise (pitch/interval varies with speed), menu interactions, game over, and new high score.
 *
 * --- Implementation Details ---
 *
 * -    Vec2D Struct: Custom 2D vector struct with overloaded operators for vector math.
 * -    State Machine: Game logic managed via a simple GameState enum (MAIN_MENU, OPTIONS_MENU, PLAYING, GAME_OVER).
 * -    Scrolling Options Menu: Options menu supports more items than can fit on screen, with scrolling indicators.
 * -    Smooth Camera Option: Provides togglable camera smoothing for player following.
 * -    Color Fading/Blending: Helper functions `fadeColor` and `blendColor` for smooth visual transitions and effects.
 * -    Sprite Rendering: Uses `pushImageRotateZoomWithAA` for anti-aliased sprite rendering with rotation and transparency.
 * -    Background Management: Background elements (stars, meteors) wrap around or reset intelligently based on camera movement and parallax factors.
 * -    Collision Handling: Differentiates between wheel collisions (resolved with physics response) and chassis/rider collisions (triggering game over).
 *
 */
 
#include <M5Cardputer.h>
#include <Preferences.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <SPI.h>

// --- Configuration ---

// Colors (Standard M5GFX/LovyanGFX 565 format)
#define BLACK       0x0000
#define NAVY        0x000F      // Used for Nebula 
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0      // Default terrain fill
#define LIGHTGREY   0xD69A
#define DARKGREY    0x7BEF
#define BLUE        0x001F      // Used for Nebula 
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0      // Used for terrain lines
#define WHITE       0xFFFF
#define ORANGE      0xFDA0      // Used for Nebula 
#define GREENYELLOW 0xB7E0
#define PINK        0xFE19
#define VIOLET      PURPLE      // Alias for Nebula 
#define DIRT_YELLOW 0xD5C0
#define NEBULA_VIOLET VIOLET
#define NEBULA_ORANGE ORANGE
#define NEBULA_BLUE   BLUE
#define NEBULA_BG     BLACK
#define TIRE_COLOR    DARKGREY
#define SPOKE_COLOR   LIGHTGREY
#define RIDER_COLOR   ORANGE
#define TERRAIN_SURFACE_COLOR YELLOW // Color for the top line of terrain
#define METEOR_COLOR  LIGHTGREY // : Color for meteors
#define _ 0 // Alias for BLACK (transparent color in sprite data)

// Physics & Gameplay Constants
/* const */ float GRAVITY = 100.0f; // Default Gravity (Editable via Options)
/* const */ float DRIVE_TORQUE = 5000.0f; // Default Drive Torque (Editable via Options)
const float MAX_DELTA_TIME = 0.03f; // Maximum time step to prevent physics instability
const float LEAN_TORQUE = 5000.0f; // Torque applied when leaning
const float WHEEL_RADIUS = 6.0f; // Visual and physics radius of wheels
const float TIRE_THICKNESS = 1.5f; // Visual thickness of tires
const float CHASSIS_MASS = 2.0f; // Mass of the main chassis
const float CHASSIS_INERTIA = 550.0f; // Rotational inertia of the chassis
const float WHEEL_MASS = 0.5f; // Mass of each wheel
const float SUSPENSION_LENGTH = 12.0f; // Natural rest length of suspension
const float SUSPENSION_STIFFNESS = 1400.0f; // Spring stiffness
const float SUSPENSION_DAMPING = 70.0f;   // Damping factor
const float SUSPENSION_MAX_FORCE = 10000.0f; // Limit suspension force
const float SUSPENSION_MAX_STRETCH_FACTOR = 1.15f; // Max length relative to rest length
const float SUSPENSION_MIN_COMPRESS_FACTOR = 0.80f; // Min length relative to rest length
const float FRICTION_COEFFICIENT = 0.5f; // Friction between wheels and ground
const float AIR_RESISTANCE_LINEAR = 0.02f; // Linear air drag (grounded)
const float AIR_RESISTANCE_ANGULAR = 30.0f; // Angular air drag (airborne)
const float MAX_ANGULAR_VELOCITY = 15.0f; // Max rotation speed (rad/s)
const float CHASSIS_COLLISION_Y_OFFSET = 2.0f; // Offset for chassis collision points
const float CHASSIS_COLLISION_X_FACTOR = 0.4f; // Horizontal factor for chassis collision points
const int PHYSICS_SUBSTEPS = 24; // Number of physics substeps per frame
const float VISUAL_ACCEL_SPIN_RATE = 5.0f; // Extra visual spin speed when accelerating
const float RIDER_LEAN_ANGLE_DEG = 45.0f; // Max visual lean angle for rider sprite
const float RIDER_HEAD_OFFSET_Y = -15.0f; // Y offset for rider head collision check

// Editable Value Steps & Limits for Options Menu
const float GRAVITY_STEP = 5.0f;
const float MIN_GRAVITY = 20.0f;
const float MAX_GRAVITY = 400.0f;
const float TORQUE_STEP = 100.0f;
const float MIN_TORQUE = 500.0f;
const float MAX_TORQUE = 10000.0f;
const float VOLUME_STEP = 5.0f;
const float BRIGHTNESS_STEP = 5.0f;

// Camera Settings
const float CAMERA_Y_FOLLOW_FACTOR = 0.1f; // Smoothing factor for vertical camera movement
const float CAMERA_Y_OFFSET_FACTOR = 0.6f; // Vertical position of player on screen

// Terrain Generation & Checkpoints
const float TERRAIN_SEGMENT_LENGTH = 30.0f; // Horizontal length of each terrain segment
const int MAX_TERRAIN_SEGMENTS = 120; // Max number of segments stored
const float TERRAIN_GENERATION_DISTANCE = 250.0f; // How far ahead to generate terrain
const float MIN_TERRAIN_HEIGHT_CHANGE = -20.0f; // Min vertical change per segment
const float MAX_TERRAIN_HEIGHT_CHANGE = 20.0f; // Max vertical change per segment
const float INITIAL_TERRAIN_HEIGHT = 100.0f; // Starting height offset from bottom
const float TERRAIN_GENERATION_AHEAD_FACTOR = 1.5f; // Generate 1.5 screens ahead
const float CHECKPOINT_INTERVAL = 1000.0f; // Distance between checkpoint flags
const int CHECKPOINT_FLAG_HEIGHT = 10; // Height of the flag pole
const int CHECKPOINT_FLAG_WIDTH = 5;   // Width of the flag triangle
const int CHECKPOINT_TEXT_Y_OFFSET = 12; // Vertical offset for distance text above flag
const float DISTANCE_SCALE_FACTOR = 10.0f; // Divide raw distance by this for display

// Terrain Color Options
const int NUM_TERRAIN_COLORS = 6;
const uint16_t terrainColors[NUM_TERRAIN_COLORS] = {
    OLIVE, DARKGREEN, MAROON, NAVY, PURPLE, DARKCYAN
};
const char* terrainColorNames[NUM_TERRAIN_COLORS] = {
    "Olive", "DkGreen", "Maroon", "Navy", "Purple", "DkCyan"
};

// Background Settings Adjustments
const float STARFIELD_TO_NEBULA_START = 1000.0f; // Start fade at 1000m
const float STARFIELD_TO_NEBULA_END = 1100.0f;   // End fade at 1100m
const float NEBULA_MAX_FADE = 0.5f;             // Max opacity for Nebula 
const float NEBULA_TO_METEOR_START = 2000.0f;    // Scaled distance to start Meteor fade-in (Nebula fade-out)
const float NEBULA_TO_METEOR_END = 2100.0f;      // Scaled distance Meteor fully in (Nebula out)
const float METEOR_TO_STARFIELD_START = 3000.0f; // Scaled distance to start Meteor fade-out
const float METEOR_TO_STARFIELD_END = 3100.0f;   // Scaled distance Meteor fully out
const float BACKGROUND_FADE_DISTANCE = 100.0f;   // Scaled distance over which fades happen (used implicitly by thresholds)


// Background Starfield
const int NUM_STARS = 75; // Number of stars
const float MIN_STAR_BRIGHTNESS = 60.0f; // Min brightness value (0-255)
const float MAX_STAR_BRIGHTNESS = 240.0f; // Max brightness value
const float MIN_PARALLAX_FACTOR = 0.1f; // Min parallax scroll speed factor
const float MAX_PARALLAX_FACTOR = 0.6f; // Max parallax scroll speed factor
const float STARFIELD_SCROLL_SPEED = 5.0f; // Base scroll speed for starfield in menus

// Background Nebula  Colors Updated
struct NebulaWave { float amplitude; float frequency; float phase; float verticalOffset; float parallaxFactor; uint16_t color; };
const int NUM_NEBULA_WAVES = 3;
NebulaWave nebulaWaves[NUM_NEBULA_WAVES] = {
    { 25.0f, 0.015f, 1.0f, 80.0f, 0.20f, NEBULA_VIOLET }, // Violet
    { 20.0f, 0.020f, 3.5f, 95.0f, 0.35f, NEBULA_ORANGE }, // Orange
    { 15.0f, 0.025f, 0.0f, 110.0f, 0.50f, NEBULA_BLUE }   // Blue
};

// Background Meteor Shower
const int NUM_METEORS = 30; // Number of meteors
const float MIN_METEOR_SPEED = 35.0f;  // Slower Min Speed
const float MAX_METEOR_SPEED = 90.0f;  // Slower Max Speed
const float MIN_METEOR_PARALLAX = 0.4f;
const float MAX_METEOR_PARALLAX = 0.9f;
const float METEOR_LENGTH = 15.0f; // Visual length of meteor trail
const float METEOR_ANGLE_DEG = -30.0f; // Angle of meteor fall (degrees from horizontal)


// Menu Settings
const int MAIN_MENU_ITEMS = 2; // Play, Options
const int OPTIONS_MENU_ITEMS = 8; // Brightness, Vol, Gravity, Torque, Info, SmoothCam, TerrainColor, MenuBG ()
const int MAX_OPTIONS_VISIBLE = 6; // Max items displayed in options menu at once
const unsigned long MENU_FADE_IN_MS = 350; // Duration for menu fade-in animation
const unsigned long MENU_NAV_COOLDOWN = 150; // Cooldown between menu navigation inputs
const unsigned long MENU_ADJUST_COOLDOWN = 150; // Cooldown between value adjustment inputs
const float MENU_TITLE_ROTATION_SPEED = 15.0f; // Speed for title rotation effect
const float MENU_TITLE_MAX_ANGLE = 5.0f; // Max angle for title rotation
const int MENU_SELECTION_PADDING = 2; // Padding around selected menu item highlight
const float MENU_BG_SCROLL_SPEED = 3.0f; // Slower scroll for menu backgrounds 

// Sound Settings
const float ENGINE_NOTE_FREQ = 600.0f; // Base frequency for engine sound
const uint32_t ENGINE_NOTE_DURATION = 8; // Duration of each engine sound pulse
const unsigned long MIN_SOUND_INTERVAL = 10; // Min ms between engine sound pulses
const unsigned long MAX_SOUND_INTERVAL = 200; // Max ms between engine sound pulses
const float MAX_SPEED_FOR_SOUND = 150.0f; // Speed at which engine sound interval is minimal
const float HIGHSCORE_SOUND_FREQ1 = 880.0f; // First note of high score sound
const uint32_t HIGHSCORE_SOUND_DUR1 = 80;
const float HIGHSCORE_SOUND_FREQ2 = 1046.0f; // Second note of high score sound
const uint32_t HIGHSCORE_SOUND_DUR2 = 100;
const uint32_t HIGHSCORE_SOUND_DELAY = 90; // Delay between high score notes
const float GAMEOVER_SOUND_FREQ = 300.0f; // Frequency for game over sound
const uint32_t GAMEOVER_SOUND_DUR = 150; // Duration for game over sound
const float MENU_NAV_SOUND_FREQ = 250.0f; // Frequency for menu navigation sound
const uint32_t MENU_NAV_SOUND_DUR = 50; // Duration for menu navigation sound
const float MENU_SEL_SOUND_FREQ = 440.0f; // Frequency for menu selection sound
const uint32_t MENU_SEL_SOUND_DUR = 75; // Duration for menu selection sound

// Particle Settings
const int MAX_PARTICLES = 80; // Max number of dirt particles on screen
const float PARTICLE_LIFETIME = 0.4f; // Lifetime of particles in seconds
const float PARTICLE_GRAVITY = 80.0f; // Gravity affecting particles
const uint16_t PARTICLE_COLOR = DIRT_YELLOW; // Color of particles
const float PARTICLE_SPEED_FACTOR = 0.4f; // Factor relating player speed to particle speed
const float PARTICLE_COUNT_FACTOR = 0.08f; // Factor relating player speed to particle spawn rate
const float PARTICLE_ANGLE_SPREAD = 0.8f; // Random spread angle for particle velocity

// Info Corner Positions Enum
enum InfoCorner { INFO_OFF, INFO_TOP_LEFT, INFO_TOP_RIGHT, INFO_BOTTOM_LEFT, INFO_BOTTOM_RIGHT, INFO_CORNER_COUNT };

// --- Simple Sprite Data (UFO) ---
const int SPRITE_CHASSIS_WIDTH = 16;
const int SPRITE_CHASSIS_HEIGHT = 10;
const uint16_t sprite_chassis_frame_data[SPRITE_CHASSIS_WIDTH * SPRITE_CHASSIS_HEIGHT] = {
    _,_,_,_,_,_,WHITE,WHITE,WHITE,WHITE,_,_,_,_,_,_,
    _,_,_,WHITE,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,WHITE,_,_,_,
    _,_,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,_,_,
    _,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,_,
    WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,
    WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,
    _,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,_,
    _,_,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,_,_,
    _,_,_,WHITE,WHITE,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,DARKGREY,WHITE,WHITE,_,_,_,
    _,_,_,_,_,_,WHITE,WHITE,WHITE,WHITE,_,_,_,_,_,_,
};

// --- Rider Sprite Data ---
const int SPRITE_RIDER_WIDTH = 10;
const int SPRITE_RIDER_HEIGHT = 12;
const uint16_t sprite_rider_data[SPRITE_RIDER_WIDTH * SPRITE_RIDER_HEIGHT] = {
    _,_,_,_,WHITE,WHITE,_,_,_,_, // Head
    _,_,_,WHITE,WHITE,WHITE,WHITE,_,_,_,
    _,_,_,WHITE,WHITE,WHITE,WHITE,_,_,_,
    _,_,_,_,RIDER_COLOR,RIDER_COLOR,_,_,_,_, // Torso
    _,_,_,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,_,_,_,
    _,_,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,_,_,
    _,_,_,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,_,_,_, // Mid Torso/Legs
    _,_,_,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,RIDER_COLOR,_,_,_,
    _,_,RIDER_COLOR,RIDER_COLOR,_,_,RIDER_COLOR,RIDER_COLOR,_,_, // Legs Split
    _,_,RIDER_COLOR,RIDER_COLOR,_,_,RIDER_COLOR,RIDER_COLOR,_,_,
    _,RIDER_COLOR,RIDER_COLOR,_,_,_,_,RIDER_COLOR,RIDER_COLOR,_, // Feet
    _,RIDER_COLOR,RIDER_COLOR,_,_,_,_,RIDER_COLOR,RIDER_COLOR,_,
};


// --- Game State ---
enum GameState { MAIN_MENU, OPTIONS_MENU, PLAYING, GAME_OVER };
GameState currentState;

// Background Type Enum 
enum BackgroundType { STARFIELD, NEBULA, METEOR_SHOWER, BACKGROUND_TYPE_COUNT };
BackgroundType currentBackgroundType = STARFIELD;
float nebulaFadeFactor = 0.0f; // Opacity of nebula layer (0.0 to NEBULA_MAX_FADE)
float meteorFadeFactor = 0.0f; // Opacity of meteor layer (0.0 to 1.0) 

// Menu Background Enum 
enum MenuBackground { MENU_STARFIELD, MENU_NEBULA, MENU_METEOR, MENU_BACKGROUND_COUNT };
MenuBackground currentMenuBackground = MENU_STARFIELD;

// --- Helper Structs ---

// 2D Vector Structure
struct Vec2D {
    float x = 0.0f; float y = 0.0f;
    // Overloaded operators for vector math
    Vec2D operator+(const Vec2D& other) const { return {x + other.x, y + other.y}; }
    Vec2D operator-(const Vec2D& other) const { return {x - other.x, y - other.y}; }
    Vec2D operator*(float scalar) const { return {x * scalar, y * scalar}; }
    Vec2D operator/(float scalar) const {
        if (abs(scalar) < std::numeric_limits<float>::epsilon()) return {0, 0}; // Avoid division by zero
        return {x / scalar, y / scalar};
    }
    Vec2D& operator+=(const Vec2D& other) { x += other.x; y += other.y; return *this; }
    Vec2D& operator-=(const Vec2D& other) { x -= other.x; y -= other.y; return *this; }
    Vec2D& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
    // Vector utility functions
    float length() const { return sqrt(x * x + y * y); }
    Vec2D normalized() const { float l = length(); return (l > 0) ? Vec2D{x / l, y / l} : Vec2D{0, 0}; }
    Vec2D rotated(float angle) const { float s = sin(angle); float c = cos(angle); return {x * c - y * s, x * s + y * c}; }
};
// Dot product function
float dot(const Vec2D& a, const Vec2D& b) { return a.x * b.x + a.y * b.y; }
// Rotate a point around a center
Vec2D rotatePoint(Vec2D point, Vec2D center, float angle) { return (point - center).rotated(angle) + center; }

// Terrain Segment Structure
struct TerrainSegment {
    Vec2D start; // Start point of the segment
    Vec2D end;   // End point of the segment
    Vec2D normal; // Upward-pointing normal vector
    Vec2D tangent; // Direction vector from start to end
    float length; // Length of the segment
};

// Wheel Structure
struct Wheel {
    Vec2D position; // World position
    Vec2D velocity; // World velocity
    Vec2D chassisAttachmentPointLocal; // Attachment point relative to chassis center
    float radius = WHEEL_RADIUS;
    float mass = WHEEL_MASS;
    bool onGround = false; // Is the wheel currently touching the ground?
    float currentSuspensionLength = SUSPENSION_LENGTH; // Current length of suspension
    Vec2D suspensionForce = {0, 0}; // Force applied by suspension this step
    Vec2D contactPoint = {0, 0}; // Last point of contact with terrain
    Vec2D contactNormal = {0, -1}; // Normal of the surface at the last contact point
    float angle = 0.0f; // Visual rotation angle for drawing spokes
};

// Bike Chassis Structure
struct BikeChassis {
    Vec2D position; // Center of mass position
    Vec2D velocity;
    Vec2D acceleration;
    float angle = 0.0f; // Rotation angle in radians
    float angularVelocity = 0.0f; // rad/s
    float angularAcceleration = 0.0f; // rad/s^2
    float width = SPRITE_CHASSIS_WIDTH;
    float height = SPRITE_CHASSIS_HEIGHT;
    float mass = CHASSIS_MASS;
    float inertia = CHASSIS_INERTIA; // Moment of inertia
    const uint16_t* spriteData = sprite_chassis_frame_data; // Pointer to sprite pixel data
    Wheel wheels[2]; // Array containing the two wheels
    Vec2D localSuspensionAxis[2]; // Direction of suspension force relative to chassis
    bool isAccelerating = false; // Is the player holding the accelerator?
    float riderLeanAngle = 0.0f; // Visual lean angle offset for the rider sprite
};

// Star Structure (for background)
struct Star {
    Vec2D basePosition; // Initial position (relative to a large virtual space)
    float parallaxFactor; // How much it moves relative to camera
};

// Particle Structure (for dirt effects)
struct Particle {
    Vec2D position;
    Vec2D velocity;
    float life; // Remaining lifetime in seconds
    uint16_t color;
};

// Meteor Structure 
struct Meteor {
    Vec2D position; // Current position (top-left of the meteor line)
    Vec2D velocity; // Constant velocity vector
    float parallaxFactor; // Parallax scroll speed factor
    float length; // Visual length
    uint16_t color;
};


// --- Global Variables ---
BikeChassis player; // The player's vehicle object
std::vector<TerrainSegment> terrainSegments; // Dynamic list of terrain segments
std::vector<Star> stars; // List of background stars
std::vector<Particle> particles; // List of active dirt particles
std::vector<Meteor> meteors; // List of active meteors 
float screenWidth = 0, screenHeight = 0; // Display dimensions
unsigned long lastUpdateTime = 0; // Timestamp of the last frame update
float deltaTime = 0.0f; // Time elapsed since the last frame in seconds
M5Canvas canvas(&M5Cardputer.Display); // Off-screen buffer for drawing
M5Canvas titleSprite(&canvas); // Sprite for rotating menu titles
M5Canvas gameOverTitleSprite(&canvas); // Sprite for rotating game over title
float lastTerrainX = 0.0f; // X coordinate of the end of the last generated terrain segment
float lastTerrainY = 0.0f; // Y coordinate of the end of the last generated terrain segment
float cameraOffsetX = 0.0f; // Camera's horizontal offset from origin
float cameraOffsetY = 0.0f; // Camera's vertical offset from origin
float targetCameraOffsetX = 0.0f; // Target offset for smooth camera movement
float targetCameraOffsetY = 0.0f; // Target offset for smooth camera movement
int selectedMenuItem = 0; // Currently selected item in the main menu
int selectedOptionItem = 0; // Currently selected item in the options menu
int optionsMenuScrollOffset = 0; // Index of the top visible item in options menu
float brightnessOption = 50.0f; // Brightness setting percentage
float soundVolume = 25.0f; // Sound volume setting percentage
InfoCorner infoCornerPosition = INFO_TOP_LEFT; // Position for FPS/Dist display
int currentTerrainColorIndex = 0; // Index into terrainColors array
bool smoothCameraEnabled = true; // Is smooth camera enabled?
unsigned long menuEntryTime = 0; // Timestamp when entering a menu (for fade-in)
unsigned long lastInteractionTime = 0; // Timestamp of last user input
unsigned long lastMenuNavTime = 0; // Timestamp for menu navigation cooldown
unsigned long lastMenuAdjustTime = 0; // Timestamp for menu value adjustment cooldown
unsigned long lastSoundPlayTime = 0; // Timestamp for engine sound interval control
float menuBackgroundOffset = 0.0f; // Horizontal offset for scrolling menu background 
float menuTitleAngle = 0.0f; // Current rotation angle for menu title
int menuTitleAngleDir = 1; // Direction of menu title rotation
float gameOverTitleAngle = 0.0f; // Current rotation angle for game over title
int gameOverTitleAngleDir = 1; // Direction of game over title rotation
float totalDistanceTravelled = 0.0f; // Raw total horizontal distance covered
float maxPlayerX = 0.0f; // Maximum X coordinate reached by the player
int lastCheckpointPassed = -1; // Index of the last checkpoint flag passed
float highScore = 0.0f; // Highest raw distance achieved
bool scoreSaved = false; // Flag to ensure score is saved only once per game over
bool acceptGameOverInput = false; // Flag to require key release before restarting from game over
Preferences preferences; // Object for storing persistent settings

// --- Forward Declarations ---
void resetGame();
void updatePhysics(float dt);
void updateParticles(float dt);
void updateMeteors(float dt, float currentOffsetX, float currentOffsetY);
void spawnParticles(const Wheel& wheel, float playerSpeed);
void generateTerrainIfNeeded();
void generateStars();
void generateMeteors(); // 
bool checkWheelTerrainCollision(Wheel& wheel, const TerrainSegment& segment, Vec2D& collisionPoint, Vec2D& collisionNormal);
void resolveWheelTerrainCollision(Wheel& wheel, BikeChassis& chassis, const Vec2D& collisionPoint, const Vec2D& collisionNormal, float dt);
bool checkChassisPointTerrainCollision(const Vec2D& checkPoint, const TerrainSegment& segment, Vec2D& collisionPoint, Vec2D& collisionNormal);
void checkAndResolveCollisions(float dt);
void render();
void handleInput(float dt);
void calculateSegmentVectors(TerrainSegment& segment);
Vec2D getChassisPointInWorld(const BikeChassis& chassis, const Vec2D& localPoint);
Vec2D getChassisVelocityAtPoint(const BikeChassis& chassis, const Vec2D& worldPoint);
float getTerrainHeightAt(float x);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
template<typename T> T constrain_val(T val, T min_val, T max_val);
void drawBackground();
void drawStars(float currentOffsetX, float currentOffsetY);
void drawNebula(float currentOffsetX, float currentOffsetY, float fadeFactor);
void drawMeteors(float currentOffsetX, float currentOffsetY, float fadeFactor); // 
const char* getInfoCornerName(InfoCorner corner);
const char* getOnOffName(bool value);
const char* getTerrainColorName(int index);
const char* getMenuBackgroundName(MenuBackground bg); // 
void applyDriveForce(Wheel& wheel, float torqueFraction);
uint16_t fadeColor(uint16_t targetColor, uint16_t startColor, float factor);
uint16_t blendColor(uint16_t color1, uint16_t color2, float factor);

// --- Helper Function Definitions ---

/* @brief Maps a value from one range to another.
 */
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Avoid division by zero
  if (abs(in_max - in_min) < std::numeric_limits<float>::epsilon()) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Constrains a value between a minimum and maximum.
 * @tparam T Data type of the value.
 */
template<typename T>
T constrain_val(T val, T min_val, T max_val) {
    return std::min(max_val, std::max(min_val, val));
}

/**
 * @brief Gets the display name for an InfoCorner enum value.
 */
const char* getInfoCornerName(InfoCorner corner) {
    switch(corner) {
        case INFO_OFF: return "OFF";
        case INFO_TOP_LEFT: return "Top L";
        case INFO_TOP_RIGHT: return "Top R";
        case INFO_BOTTOM_LEFT: return "Bot L";
        case INFO_BOTTOM_RIGHT: return "Bot R";
        default: return "???";
    }
}

/**
 * @brief Gets the display name ("ON" or "OFF") for a boolean value.
 */
const char* getOnOffName(bool value) {
    return value ? "ON" : "OFF";
}

/* @brief Gets the display name for a terrain color index.
 */
const char* getTerrainColorName(int index) {
    if (index >= 0 && index < NUM_TERRAIN_COLORS) {
        return terrainColorNames[index];
    }
    return "???";
}

/** : Added
 * @brief Gets the display name for a MenuBackground enum value.
 */
const char* getMenuBackgroundName(MenuBackground bg) {
    switch(bg) {
        case MENU_STARFIELD: return "Stars";
        case MENU_NEBULA: return "Nebula";
        case MENU_METEOR: return "Meteors";
        default: return "???";
    }
}


/**
 * @brief Interpolates between two 565 colors.
 * @param factor Interpolation factor (0.0 = startColor, 1.0 = targetColor).
 */
uint16_t fadeColor(uint16_t targetColor, uint16_t startColor, float factor) {
    factor = constrain_val(factor, 0.0f, 1.0f);
    // Extract RGB components (5-bit R, 6-bit G, 5-bit B)
    uint8_t r1 = (startColor >> 11) & 0x1F;
    uint8_t g1 = (startColor >> 5) & 0x3F;
    uint8_t b1 = startColor & 0x1F;
    uint8_t r2 = (targetColor >> 11) & 0x1F;
    uint8_t g2 = (targetColor >> 5) & 0x3F;
    uint8_t b2 = targetColor & 0x1F;
    // Interpolate each component
    uint8_t r = r1 + (int)((r2 - r1) * factor);
    uint8_t g = g1 + (int)((g2 - g1) * factor);
    uint8_t b = b1 + (int)((b2 - b1) * factor);
    // Recombine into 565 format
    return (r << 11) | (g << 5) | b;
}

/**
 * @brief Blends two 565 colors.
 * @param factor Blend factor (0.0 = color1, 1.0 = color2).
 */
uint16_t blendColor(uint16_t color1, uint16_t color2, float factor) {
    factor = constrain_val(factor, 0.0f, 1.0f);
    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;
    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;
    // Blend components
    uint8_t r = r1 * (1.0f - factor) + r2 * factor;
    uint8_t g = g1 * (1.0f - factor) + g2 * factor;
    uint8_t b = b1 * (1.0f - factor) + b2 * factor;
    return (r << 11) | (g << 5) | b;
}

/**
 * @brief Calculates the tangent and normal vectors for a terrain segment.
 */
void calculateSegmentVectors(TerrainSegment& segment) {
    Vec2D delta = segment.end - segment.start;
    segment.length = delta.length();
    if (segment.length > 0.001f) {
        segment.tangent = delta / segment.length;
        // Normal is perpendicular to tangent, pointing generally upwards (negative Y)
        segment.normal = {-segment.tangent.y, segment.tangent.x};
        if (segment.normal.y > 0) { // Ensure normal points upwards
            segment.normal *= -1.0f;
        }
    } else { // Handle degenerate segment
        segment.tangent = {1, 0};
        segment.normal = {0, -1};
    }
}

/**
 * @brief Converts a point from chassis-local coordinates to world coordinates.
 */
Vec2D getChassisPointInWorld(const BikeChassis& chassis, const Vec2D& localPoint) {
    return chassis.position + localPoint.rotated(chassis.angle);
}

/**
 * @brief Calculates the world velocity of a point attached to the chassis.
 */
Vec2D getChassisVelocityAtPoint(const BikeChassis& chassis, const Vec2D& worldPoint) {
    Vec2D radiusVector = worldPoint - chassis.position;
    // v = v_linear + omega x r (2D equivalent: omega * (-ry, rx))
    Vec2D angularVelocityComponent = {-chassis.angularVelocity * radiusVector.y, chassis.angularVelocity * radiusVector.x};
    return chassis.velocity + angularVelocityComponent;
}

/**
 * @brief Finds the terrain height (Y coordinate) at a given world X coordinate.
 * Returns a large value if X is outside the known terrain range.
 * NOTE: This always returns the height of the *top* line, used for physics.
 */
float getTerrainHeightAt(float x) {
    for (const auto& seg : terrainSegments) {
        // Check if x is within the horizontal bounds of the segment
        if (x >= seg.start.x && x <= seg.end.x) {
            // Handle vertical segments
            if (abs(seg.end.x - seg.start.x) < 0.01f) {
                 return std::min(seg.start.y, seg.end.y); // Return top Y
            }
            // Linear interpolation for non-vertical segments
            float t = (x - seg.start.x) / (seg.end.x - seg.start.x);
            return seg.start.y + t * (seg.end.y - seg.start.y);
        }
    }
    // Handle cases where x is outside the generated terrain range
     if (!terrainSegments.empty()) {
         if (x < terrainSegments.front().start.x) { return terrainSegments.front().start.y; } // Use first point's height
         if (x > terrainSegments.back().end.x) { return terrainSegments.back().end.y; } // Use last point's height
     }
    // Fallback if no terrain exists or x is somehow still outside
    return screenHeight * 3; // Return a very large Y value (far below screen)
}

/**
 * @brief Checks for collision between a wheel (circle) and a terrain segment (line).
 * NOTE: This always checks against the *top* line of the segment.
 */
bool checkWheelTerrainCollision(Wheel& wheel, const TerrainSegment& segment, Vec2D& collisionPoint, Vec2D& collisionNormal) {
    Vec2D lineDir = segment.tangent;
    Vec2D startToCenter = wheel.position - segment.start;
    // Project wheel center onto the line defined by the segment
    float t = dot(startToCenter, lineDir);
    Vec2D closestPointOnLine;
    // Find the closest point on the *segment* (clamp t between 0 and segment length)
    if (t <= 0) { closestPointOnLine = segment.start; }
    else if (t >= segment.length) { closestPointOnLine = segment.end; }
    else { closestPointOnLine = segment.start + lineDir * t; }
    // Check distance squared between wheel center and closest point
    Vec2D centerToClosest = closestPointOnLine - wheel.position;
    float distanceSq = dot(centerToClosest, centerToClosest);
    // Collision if distance squared is less than or equal to radius squared
    if (distanceSq <= wheel.radius * wheel.radius) {
        float distance = sqrt(distanceSq);
        // Calculate collision normal (points from collision point towards wheel center)
        if (distance > 0.001f) {
            collisionNormal = (wheel.position - closestPointOnLine).normalized();
        } else { // Handle case where center is exactly on the line
            collisionNormal = segment.normal * -1.0f; // Use terrain normal (flipped)
        }
        collisionPoint = closestPointOnLine; // Point of collision
        return true;
    }
    return false; // No collision
}

/**
 * @brief Checks for collision between a single point (e.g., chassis corner, rider head) and a terrain segment.
 * NOTE: This always checks against the *top* line of the segment.
 */
bool checkChassisPointTerrainCollision(const Vec2D& checkPoint, const TerrainSegment& segment, Vec2D& collisionPoint, Vec2D& collisionNormal) {
    float segStartX = segment.start.x; float segEndX = segment.end.x;
    float segStartY = segment.start.y; float segEndY = segment.end.y;
    float yTolerance = 1.5f; // How far below the line the point can be
    float xTolerance = 1.0f; // How far past segment ends point X can be

    // Broad phase X check
    if (checkPoint.x >= std::min(segStartX, segEndX) - xTolerance && checkPoint.x <= std::max(segStartX, segEndX) + xTolerance) {
        float lineY;
        if (abs(segEndX - segStartX) < 0.01f) { // Vertical segment
            // Check Y bounds and proximity to vertical line
            if (checkPoint.y >= std::min(segStartY, segEndY) && checkPoint.y <= std::max(segStartY, segEndY)) {
                 if (abs(checkPoint.x - segStartX) < yTolerance) {
                     collisionPoint = {segStartX, checkPoint.y};
                     collisionNormal = segment.normal * -1.0f; // Normal pointing away from terrain
                     return true;
                 }
            }
        } else { // Non-vertical segment
            // Calculate terrain Y at the point's X
            float t = (checkPoint.x - segStartX) / (segEndX - segStartX);
            lineY = segStartY + t * (segEndY - segStartY);
            // Check if point is below or very close to the line
            if (checkPoint.y >= lineY - yTolerance) {
                 // Additional check: Ensure the projected point 't' is reasonably within segment bounds
                 if (t >= -0.1f && t <= 1.1f) { // Allow slight overshoot
                     collisionPoint = {checkPoint.x, lineY};
                     collisionNormal = segment.normal * -1.0f; // Normal pointing away from terrain
                     return true;
                 }
            }
        }
    }
    return false; // No collision
}
// --- END Helper Function Definitions ---


// --- Game Reset Function ---
// Resets game state, terrain, player position, etc.
void resetGame() {
    terrainSegments.clear();
    particles.clear();
    meteors.clear(); // 
    currentBackgroundType = STARFIELD; // Start with stars
    nebulaFadeFactor = 0.0f;
    meteorFadeFactor = 0.0f; // 
    lastTerrainX = -TERRAIN_GENERATION_DISTANCE; // Start generating before screen edge
    lastTerrainY = screenHeight - INITIAL_TERRAIN_HEIGHT;

    // Initial flat segment to start on
    Vec2D startPoint = {lastTerrainX, lastTerrainY};
    Vec2D endPoint = {lastTerrainX + TERRAIN_SEGMENT_LENGTH * 2, lastTerrainY};
    TerrainSegment initialSegment = {startPoint, endPoint};
    calculateSegmentVectors(initialSegment);
    terrainSegments.push_back(initialSegment);
    lastTerrainX = endPoint.x;
    lastTerrainY = endPoint.y;

    // Generate initial terrain ahead
    float initialGenerationTargetX = screenWidth * 1.5f;
    int initialSegmentsGenerated = 0;
    while(lastTerrainX < initialGenerationTargetX) {
        float heightChange = random(MIN_TERRAIN_HEIGHT_CHANGE * 100, MAX_TERRAIN_HEIGHT_CHANGE * 100) / 100.0f;
        float nextY = lastTerrainY + heightChange;
        nextY = constrain_val(nextY, 50.0f, screenHeight * 1.5f); // Prevent extreme heights
        Vec2D segStart = {lastTerrainX, lastTerrainY};
        Vec2D segEnd = {lastTerrainX + TERRAIN_SEGMENT_LENGTH, nextY};
        TerrainSegment newSegment = {segStart, segEnd};
        calculateSegmentVectors(newSegment);
        terrainSegments.push_back(newSegment);
        lastTerrainX = segEnd.x;
        lastTerrainY = segEnd.y;
        initialSegmentsGenerated++;
        if (initialSegmentsGenerated > MAX_TERRAIN_SEGMENTS * 3) break; // Safety break
    }

    // Reset Player State
    float spawnX = screenWidth / 4.0f;
    float spawnTerrainY = getTerrainHeightAt(spawnX);
    float spawnY = spawnTerrainY - SUSPENSION_LENGTH - WHEEL_RADIUS - player.height - 15.0f;
    if (spawnTerrainY >= screenHeight * 3) { // Fallback if terrain height invalid
        spawnY = screenHeight - INITIAL_TERRAIN_HEIGHT - SUSPENSION_LENGTH - WHEEL_RADIUS - player.height - 15.0f;
    }
    player.position = {spawnX, spawnY};
    player.velocity = {20, 0}; player.angle = 0.0f; player.angularVelocity = 0.0f;
    player.acceleration = {0, 0}; player.angularAcceleration = 0.0f;
    player.isAccelerating = false; player.riderLeanAngle = 0.0f;

    // Reset Wheels
    player.localSuspensionAxis[0] = Vec2D{-0.8f, 1.0f}.normalized();
    player.localSuspensionAxis[1] = Vec2D{ 0.8f, 1.0f}.normalized();
    player.wheels[0].chassisAttachmentPointLocal = {-SPRITE_CHASSIS_WIDTH * 0.35f, 3.5f};
    player.wheels[1].chassisAttachmentPointLocal = { SPRITE_CHASSIS_WIDTH * 0.35f, 3.5f};
    for (int i = 0; i < 2; ++i) {
        Vec2D suspensionAxisWorld = player.localSuspensionAxis[i].rotated(player.angle);
        Vec2D suspensionAttachWorld = getChassisPointInWorld(player, player.wheels[i].chassisAttachmentPointLocal);
        player.wheels[i].position = suspensionAttachWorld + suspensionAxisWorld * SUSPENSION_LENGTH;
        player.wheels[i].velocity = player.velocity; player.wheels[i].onGround = false;
        player.wheels[i].currentSuspensionLength = SUSPENSION_LENGTH;
        player.wheels[i].suspensionForce = {0, 0}; player.wheels[i].angle = 0.0f;
    }

    // Reset Camera
    targetCameraOffsetX = player.position.x - screenWidth / 3.0f;
    targetCameraOffsetY = player.position.y - screenHeight * CAMERA_Y_OFFSET_FACTOR;
    cameraOffsetX = targetCameraOffsetX; cameraOffsetY = targetCameraOffsetY;

    // Reset Stats & Flags
    totalDistanceTravelled = 0.0f; maxPlayerX = player.position.x; lastCheckpointPassed = -1;
    scoreSaved = false; acceptGameOverInput = false; gameOverTitleAngle = 0.0f; gameOverTitleAngleDir = 1;
    lastUpdateTime = 0; deltaTime = 0.0f;

    // Regenerate meteors for the new game
    generateMeteors();
}

// --- Cardputer Setup Function ---
// Initializes the M5 Cardputer, display, speaker, loads settings, and sets initial game state.
void setup() {
    auto cfg = M5.config(); // Get default config
    M5Cardputer.begin(cfg); // Initialize Cardputer with config
    M5.Speaker.begin();     // Initialize speaker

    // Get screen dimensions and create drawing buffer
    screenWidth = M5Cardputer.Display.width();
    screenHeight = M5Cardputer.Display.height();
    canvas.createSprite(screenWidth, screenHeight);
    M5Cardputer.Display.setRotation(1); // Set landscape rotation

    // Load persistent settings (read-only first)
    preferences.begin("sunrider", true);
    highScore = preferences.getFloat("highscore", 0.0f);
    brightnessOption = preferences.getFloat("bright", 50.0f);
    GRAVITY = preferences.getFloat("gravity", 100.0f);
    DRIVE_TORQUE = preferences.getFloat("torque", 5000.0f);
    soundVolume = preferences.getFloat("volume", 25.0f);
    infoCornerPosition = (InfoCorner)preferences.getInt("infocorn", INFO_TOP_LEFT);
    smoothCameraEnabled = preferences.getBool("smoothcam", true);
    currentTerrainColorIndex = preferences.getInt("terraincolor", 0);
    currentMenuBackground = (MenuBackground)preferences.getInt("menubg", MENU_STARFIELD); //  Load menu BG
    preferences.end();

    // Apply loaded settings and constrain values
    M5Cardputer.Display.setBrightness((uint8_t)constrain_val(brightnessOption * 2.55f, 0.0f, 255.0f));
    M5.Speaker.setVolume((uint8_t)mapfloat(soundVolume, 0.0f, 100.0f, 0.0f, 255.0f));
    GRAVITY = constrain_val(GRAVITY, MIN_GRAVITY, MAX_GRAVITY);
    DRIVE_TORQUE = constrain_val(DRIVE_TORQUE, MIN_TORQUE, MAX_TORQUE);
    currentTerrainColorIndex = constrain_val(currentTerrainColorIndex, 0, NUM_TERRAIN_COLORS - 1);
    currentMenuBackground = (MenuBackground)constrain_val((int)currentMenuBackground, 0, (int)MENU_BACKGROUND_COUNT - 1); //  Constrain menu BG

    M5Cardputer.Display.fillScreen(BLACK); // Clear physical screen initially

    // Initialize player physics properties
    player.mass = CHASSIS_MASS; player.inertia = CHASSIS_INERTIA;
    player.spriteData = sprite_chassis_frame_data;
    player.width = SPRITE_CHASSIS_WIDTH; player.height = SPRITE_CHASSIS_HEIGHT;
    for (int i = 0; i < 2; ++i) { player.wheels[i].radius = WHEEL_RADIUS; player.wheels[i].mass = WHEEL_MASS; }

    // Initialize game state and timers
    generateStars(); // Create initial starfield
    generateMeteors(); // Create initial meteor pool 
    currentBackgroundType = STARFIELD; nebulaFadeFactor = 0.0f; meteorFadeFactor = 0.0f;
    currentState = MAIN_MENU; menuEntryTime = millis(); // Start in main menu
    lastInteractionTime = millis(); lastMenuNavTime = millis(); lastMenuAdjustTime = millis(); lastSoundPlayTime = millis();
    menuTitleAngle = 0.0f; menuTitleAngleDir = 1; gameOverTitleAngle = 0.0f; gameOverTitleAngleDir = 1;
    optionsMenuScrollOffset = 0;
}

// --- Cardputer Loop Function ---
// Main game loop called repeatedly.
void loop() {
    M5Cardputer.update(); // Update device state (keyboard, etc.)

    // Calculate deltaTime (time since last frame)
    unsigned long currentTime = millis();
    if (lastUpdateTime == 0) { lastUpdateTime = currentTime; deltaTime = 0.0f; }
    else { deltaTime = (currentTime - lastUpdateTime) / 1000.0f; lastUpdateTime = currentTime; }
    if (deltaTime <= 0) { render(); return; } // Skip update if no time passed
    deltaTime = std::min(deltaTime, MAX_DELTA_TIME); // Clamp max delta time

    // Handle Input
    handleInput(deltaTime);

    // Update Game Logic based on State
    switch (currentState) {
        case MAIN_MENU:
        case OPTIONS_MENU:
            // Animate title and scroll menu background
            menuTitleAngle += menuTitleAngleDir * MENU_TITLE_ROTATION_SPEED * deltaTime;
            if (menuTitleAngle > MENU_TITLE_MAX_ANGLE || menuTitleAngle < -MENU_TITLE_MAX_ANGLE) { menuTitleAngleDir *= -1; menuTitleAngle = constrain_val(menuTitleAngle, -MENU_TITLE_MAX_ANGLE, MENU_TITLE_MAX_ANGLE); }
            menuBackgroundOffset += MENU_BG_SCROLL_SPEED * deltaTime; //  Use dedicated scroll speed
            // Update meteors if they are the selected menu background
            if (currentMenuBackground == MENU_METEOR) {
                if (meteors.empty()) {
                    generateMeteors(); // Regenerate the meteor pool if it was cleared
                }
                // Pass menu offsets to updateMeteors
                updateMeteors(deltaTime, menuBackgroundOffset, 0.0f);
            }
            break;

        case GAME_OVER:
            // Animate game over title and scroll background (uses game camera offset)
            gameOverTitleAngle += gameOverTitleAngleDir * MENU_TITLE_ROTATION_SPEED * deltaTime;
            if (gameOverTitleAngle > MENU_TITLE_MAX_ANGLE || gameOverTitleAngle < -MENU_TITLE_MAX_ANGLE) { gameOverTitleAngleDir *= -1; gameOverTitleAngle = constrain_val(gameOverTitleAngle, -MENU_TITLE_MAX_ANGLE, MENU_TITLE_MAX_ANGLE); }
            // Keep background elements updating based on final camera position
            // Pass game camera offsets to updateMeteors
            updateMeteors(deltaTime, cameraOffsetX, cameraOffsetY);
            break;

        case PLAYING:
        { // Scope for variables specific to PLAYING state update
            // Update background type and fade factors based on distance
            float currentScaledDist = totalDistanceTravelled / DISTANCE_SCALE_FACTOR;
            nebulaFadeFactor = 0.0f;
            meteorFadeFactor = 0.0f;
            currentBackgroundType = STARFIELD; // Default to starfield

            // Starfield Only (< 1000m)
            if (currentScaledDist < STARFIELD_TO_NEBULA_START) {
                currentBackgroundType = STARFIELD;
                // Factors are already 0.0f
            }
            // Starfield -> Nebula Fade (1000m to 1100m)
            else if (currentScaledDist >= STARFIELD_TO_NEBULA_START && currentScaledDist < STARFIELD_TO_NEBULA_END) {
                currentBackgroundType = NEBULA; // Draw Nebula during fade
                nebulaFadeFactor = mapfloat(currentScaledDist, STARFIELD_TO_NEBULA_START, STARFIELD_TO_NEBULA_END, 0.0f, NEBULA_MAX_FADE);
            }
            // Nebula Full (1100m to 2000m)
            else if (currentScaledDist >= STARFIELD_TO_NEBULA_END && currentScaledDist < NEBULA_TO_METEOR_START) {
                currentBackgroundType = NEBULA;
                nebulaFadeFactor = NEBULA_MAX_FADE;
            }
            // Nebula -> Meteor Fade (2000m to 2100m)
            else if (currentScaledDist >= NEBULA_TO_METEOR_START && currentScaledDist < NEBULA_TO_METEOR_END) {
                currentBackgroundType = METEOR_SHOWER;
                // Fade nebula out
                nebulaFadeFactor = mapfloat(currentScaledDist, NEBULA_TO_METEOR_START, NEBULA_TO_METEOR_END, NEBULA_MAX_FADE, 0.0f);
                // Fade meteor in
                meteorFadeFactor = mapfloat(currentScaledDist, NEBULA_TO_METEOR_START, NEBULA_TO_METEOR_END, 0.0f, 1.0f);
            }
            // Meteor Full (2100m to 3000m)
            else if (currentScaledDist >= NEBULA_TO_METEOR_END && currentScaledDist < METEOR_TO_STARFIELD_START) {
                currentBackgroundType = METEOR_SHOWER;
                meteorFadeFactor = 1.0f;
            }
            // Meteor -> Starfield Fade (3000m to 3100m)
            else if (currentScaledDist >= METEOR_TO_STARFIELD_START && currentScaledDist < METEOR_TO_STARFIELD_END) {
                currentBackgroundType = METEOR_SHOWER; // Still draw meteors while fading out
                meteorFadeFactor = mapfloat(currentScaledDist, METEOR_TO_STARFIELD_START, METEOR_TO_STARFIELD_END, 1.0f, 0.0f);
            }
            // Starfield only (after 3100m)
            else if (currentScaledDist >= METEOR_TO_STARFIELD_END) {
                currentBackgroundType = STARFIELD;
                // Factors already 0.0f
            }

            // Ensure factors are clamped
            nebulaFadeFactor = constrain_val(nebulaFadeFactor, 0.0f, NEBULA_MAX_FADE);
            meteorFadeFactor = constrain_val(meteorFadeFactor, 0.0f, 1.0f);


            // Physics Update with Substepping
            const int substeps = PHYSICS_SUBSTEPS;
            float subDeltaTime = deltaTime / substeps;
            for (int i = 0; i < substeps; ++i) {
                updatePhysics(subDeltaTime); // Update positions, velocities, forces
                if (currentState == PLAYING) {
                    checkAndResolveCollisions(subDeltaTime); // Check and handle collisions
                    if (currentState == GAME_OVER) break; // Exit substep loop if game ended
                } else { break; } // Exit if state changed
            }

            // Post-Physics Updates (if still playing)
            if (currentState == PLAYING) {
                updateParticles(deltaTime); // Update particle positions and lifetimes
                // Pass game camera offsets to updateMeteors
                updateMeteors(deltaTime, cameraOffsetX, cameraOffsetY);
                generateTerrainIfNeeded(); // Generate new terrain segments

                // Update Camera Position
                targetCameraOffsetX = player.position.x - screenWidth / 3.0f;
                targetCameraOffsetY = player.position.y - screenHeight * CAMERA_Y_OFFSET_FACTOR;
                if (smoothCameraEnabled) { // Apply smoothing if enabled
                    cameraOffsetX += (targetCameraOffsetX - cameraOffsetX) * 0.2f;
                    cameraOffsetY += (targetCameraOffsetY - cameraOffsetY) * CAMERA_Y_FOLLOW_FACTOR;
                } else { // Snap camera instantly
                    cameraOffsetX = targetCameraOffsetX; cameraOffsetY = targetCameraOffsetY;
                }

                // Update Distance Counter & Checkpoints
                if (player.position.x > maxPlayerX) {
                    totalDistanceTravelled += (player.position.x - maxPlayerX);
                    maxPlayerX = player.position.x;
                    float nextCheckpointX = (float)(lastCheckpointPassed + 1) * CHECKPOINT_INTERVAL;
                    if (maxPlayerX >= nextCheckpointX) { lastCheckpointPassed++; }
                }

                // Reset forces for next frame (applied per step)
                player.acceleration = {0, 0}; player.angularAcceleration = 0.0f;
            }
        } // End PLAYING scope
        break; // End PLAYING case
    } // End switch(currentState)

    // Render the current frame
    render();
}

// --- Star Generation ---
void generateStars() {
    stars.clear(); stars.reserve(NUM_STARS); randomSeed(millis());
    for (int i = 0; i < NUM_STARS; ++i) {
        Star s;
        s.basePosition.x = random(0, (long)(screenWidth * 3)); // Spread over 3 screen widths initially
        s.basePosition.y = random(0, (long)(screenHeight * 2)); // Spread over 2 screen heights initially
        s.parallaxFactor = random(MIN_PARALLAX_FACTOR * 100, MAX_PARALLAX_FACTOR * 100) / 100.0f;
        stars.push_back(s);
    }
}

// --- Meteor Generation---
void generateMeteors() {
    meteors.clear(); meteors.reserve(NUM_METEORS); randomSeed(millis() + 100); // Different seed
    float angleRad = METEOR_ANGLE_DEG * M_PI / 180.0f;
    Vec2D baseVelDir = {cos(angleRad), sin(angleRad)};

    for (int i = 0; i < NUM_METEORS; ++i) {
        Meteor m;
        m.position.x = random(0, (long)(screenWidth * 3)); // Initial X spread

        m.position.y = random(-(long)(screenHeight * 0.75), -(long)(screenHeight * 0.1)); // Spawn closer

        m.parallaxFactor = random(MIN_METEOR_PARALLAX * 100, MAX_METEOR_PARALLAX * 100) / 100.0f;
        float speed = random(MIN_METEOR_SPEED * 100, MAX_METEOR_SPEED * 100) / 100.0f;

        // Flip the Y-component sign to make them fall downwards
        m.velocity = Vec2D{baseVelDir.x * speed, -baseVelDir.y * speed};

        m.length = METEOR_LENGTH * (random(80, 120) / 100.0f); // Slight length variation
        m.color = METEOR_COLOR;
        meteors.push_back(m);
    }
}


// --- Meteor Update Function --- Uses Offset Params
void updateMeteors(float dt, float currentOffsetX, float currentOffsetY) { // Added parameters
    float screenWidthF = (float)screenWidth;
    float screenHeightF = (float)screenHeight;
    // Define respawn boundaries slightly larger than the initial generation area
    float respawnMarginX = screenWidthF * 0.5f;
    float respawnMarginY = screenHeightF * 0.5f; // Keep Y margin for triggering reset

    for (auto& m : meteors) {
        m.position += m.velocity * dt;

        // Calculate visual position based on parallax using the PASSED offsets
        float checkX = m.position.x - currentOffsetX * m.parallaxFactor;
        float checkY = m.position.y - currentOffsetY * m.parallaxFactor;

        // Check if meteor is well outside the screen bounds (consider its length)
        bool resetPosition = false;
        // Use larger margins for reset trigger
        if (m.velocity.y > 0 && checkY - m.length > screenHeightF + respawnMarginY) { // Gone way off bottom
            resetPosition = true;
        } else if (m.velocity.y < 0 && checkY + m.length < -respawnMarginY) { // Gone way off top
            resetPosition = true;
        } else if (m.velocity.x > 0 && checkX - m.length > screenWidthF + respawnMarginX) { // Gone way off right
            resetPosition = true;
        } else if (m.velocity.x < 0 && checkX + m.length < -respawnMarginX) { // Gone way off left
            resetPosition = true;
        }

        if (resetPosition) {
            // Reset based on the direction it went off-screen.
            // Use PASSED offsets for reset positioning relative to the current view.

            if (m.velocity.y > 0 && checkY - m.length > screenHeightF + respawnMarginY) { // Went off bottom
                // Reset to top, random X relative to current view
                m.position.x = currentOffsetX * m.parallaxFactor + random(-(int)respawnMarginX, (int)(screenWidthF + respawnMarginX));
                // Reset closer above view
                // m.position.y = currentOffsetY * m.parallaxFactor - random((int)(screenHeightF * 0.1f), (int)respawnMarginY);
                 m.position.y = currentOffsetY * m.parallaxFactor - random(5, (int)(screenHeightF * 0.3f)); // Reset closer
            } else if (m.velocity.x < 0 && checkX + m.length < -respawnMarginX) { // Went off left
                // Reset to right, random Y relative to current view
                 m.position.x = currentOffsetX * m.parallaxFactor + screenWidthF + random((int)(screenWidthF * 0.1f), (int)respawnMarginX); // Reset right of view
                 // Tighter Y reset range
                 // m.position.y = currentOffsetY * m.parallaxFactor + random(-(int)respawnMarginY, (int)(screenHeightF + respawnMarginY));
                 m.position.y = currentOffsetY * m.parallaxFactor + random(-(int)(screenHeightF * 0.3f), (int)(screenHeightF + screenHeightF * 0.3f));
            } else if (m.velocity.x > 0 && checkX - m.length > screenWidthF + respawnMarginX) { // Went off right
                // Reset to left, random Y relative to current view
                 m.position.x = currentOffsetX * m.parallaxFactor - random((int)(screenWidthF * 0.1f), (int)respawnMarginX); // Reset left of view
                 // Tighter Y reset range
                 // m.position.y = currentOffsetY * m.parallaxFactor + random(-(int)respawnMarginY, (int)(screenHeightF + respawnMarginY));
                 m.position.y = currentOffsetY * m.parallaxFactor + random(-(int)(screenHeightF * 0.3f), (int)(screenHeightF + screenHeightF * 0.3f));
            } else { // Default / Went off top
                // Reset to top, random X relative to current view
                m.position.x = currentOffsetX * m.parallaxFactor + random(-(int)respawnMarginX, (int)(screenWidthF + respawnMarginX));
                 // Reset closer above view
                // m.position.y = currentOffsetY * m.parallaxFactor - random((int)(screenHeightF * 0.1f), (int)respawnMarginY);
                 m.position.y = currentOffsetY * m.parallaxFactor - random(5, (int)(screenHeightF * 0.3f)); // Reset closer
            }

            // Keep existing velocity, parallax, length, color.
            // Optionally re-randomize speed slightly:
            // float speed = random(MIN_METEOR_SPEED * 100, MAX_METEOR_SPEED * 100) / 100.0f;
            // float angleRad = METEOR_ANGLE_DEG * M_PI / 180.0f;
            // Vec2D baseVelDir = {cos(angleRad), sin(angleRad)};
            // m.velocity = Vec2D{baseVelDir.x * speed, -baseVelDir.y * speed};
        }
    }
}



// --- Draw Starfield Helper ---
void drawStars(float currentOffsetX, float currentOffsetY) {
    float screenWidthF = (float)screenWidth; float screenHeightF = (float)screenHeight;
    float wrapWidth = screenWidthF * 3.0f; // Width of the virtual space stars are in
    float wrapHeight = screenHeightF * 2.0f;

    for (const auto& star : stars) {
        // Calculate parallax-adjusted position relative to the virtual space origin
        float virtualX = star.basePosition.x - currentOffsetX * star.parallaxFactor;
        float virtualY = star.basePosition.y - currentOffsetY * star.parallaxFactor;

        // Wrap the virtual position within the defined space
        float wrappedX = fmod(virtualX, wrapWidth); if (wrappedX < 0) wrappedX += wrapWidth;
        float wrappedY = fmod(virtualY, wrapHeight); if (wrappedY < 0) wrappedY += wrapHeight;

        // Check if the wrapped position is potentially visible on screen
        // Check against a slightly larger area to avoid pop-in at edges
        if (wrappedX >= -10 && wrappedX < screenWidthF + 10 &&
            wrappedY >= -10 && wrappedY < screenHeightF + 10)
        {
            float brightness = constrain_val(mapfloat(star.parallaxFactor, MIN_PARALLAX_FACTOR, MAX_PARALLAX_FACTOR, MIN_STAR_BRIGHTNESS, MAX_STAR_BRIGHTNESS), 0.0f, 255.0f);
            uint16_t starColor = canvas.color565((uint8_t)brightness, (uint8_t)brightness, (uint8_t)brightness);
            canvas.drawPixel(wrappedX, wrappedY, starColor);
        }
    }
}

// --- Draw Nebula Helper ---  Max Fade Added
void drawNebula(float currentOffsetX, float currentOffsetY, float fadeFactor) {
    fadeFactor = constrain_val(fadeFactor, 0.0f, NEBULA_MAX_FADE); // Apply max fade limit
    if (fadeFactor <= 0.0f) return; // Nothing to draw

    std::vector<int> y_below(screenWidth, screenHeight); // Track the bottom edge of the previous layer

    for (int waveIdx = 0; waveIdx < NUM_NEBULA_WAVES; ++waveIdx) {
        const auto& wave = nebulaWaves[waveIdx];
        std::vector<int> current_wave_y(screenWidth);
        uint16_t currentWaveColor = blendColor(NEBULA_BG, wave.color, fadeFactor); // Blend with background based on fade

        for (int x = 0; x < screenWidth; ++x) {
            // Calculate world position considering parallax
            float worldX = x + currentOffsetX * wave.parallaxFactor;
            float worldY = wave.amplitude * sin(wave.frequency * worldX + wave.phase) + wave.verticalOffset;

            // Calculate screen position considering parallax
            int screenY = constrain_val((int)(worldY - currentOffsetY * wave.parallaxFactor), 0, (int)screenHeight);
            current_wave_y[x] = screenY;

            // Fill the area between the previous layer's bottom edge and this wave's top edge
            int y_start = y_below[x]; // Bottom of the layer above
            int y_end = screenY;      // Top of the current wave at this x

            if (y_start > y_end) { // Only draw if there's space (current wave is higher)
                canvas.drawFastVLine(x, y_end, y_start - y_end, currentWaveColor);
            }
        }
        // Update the bottom edge tracker for the next layer
        y_below = current_wave_y;
    }

    // Fill the area below the last wave to the bottom of the screen
    const auto& lastWave = nebulaWaves[NUM_NEBULA_WAVES - 1];
    uint16_t lastWaveColor = blendColor(NEBULA_BG, lastWave.color, fadeFactor);
     for (int x = 0; x < screenWidth; ++x) {
         int y_start = y_below[x]; // Bottom of the last drawn wave
         int y_end = screenHeight; // Bottom of the screen
         if (y_start < y_end) { // Only draw if there's space below the last wave
             canvas.drawFastVLine(x, y_start, y_end - y_start, lastWaveColor);
         }
     }
}

// --- Draw Meteors Helper  ---
void drawMeteors(float currentOffsetX, float currentOffsetY, float fadeFactor) {
    if (fadeFactor <= 0.0f) return; // Nothing to draw

    float screenWidthF = (float)screenWidth;
    float screenHeightF = (float)screenHeight;

    for (const auto& m : meteors) {
        // Calculate screen position based on parallax
        float screenX = m.position.x - currentOffsetX * m.parallaxFactor;
        float screenY = m.position.y - currentOffsetY * m.parallaxFactor;

        // Calculate tail end position based on velocity direction and length
        Vec2D dir = m.velocity.normalized();
        float tailX = screenX - dir.x * m.length;
        float tailY = screenY - dir.y * m.length;

        // Basic culling (check if line segment bounding box overlaps screen)
        float minX = std::min(screenX, tailX);
        float maxX = std::max(screenX, tailX);
        float minY = std::min(screenY, tailY);
        float maxY = std::max(screenY, tailY);

        if (maxX >= 0 && minX < screenWidthF && maxY >= 0 && minY < screenHeightF) {
            // Blend color based on fade factor (fade from transparent black)
            uint16_t drawColor = blendColor(BLACK, m.color, fadeFactor);
            canvas.drawLine(screenX, screenY, tailX, tailY, drawColor);
        }
    }
}


// --- Draw Background Function ---  Updated
void drawBackground() {
    float currentOffsetX = 0.0f;
    float currentOffsetY = 0.0f;
    float currentNebulaFade = 0.0f;
    float currentMeteorFade = 0.0f;
    MenuBackground menuBgToDraw = MENU_STARFIELD; // Default for safety

    if (currentState == PLAYING || currentState == GAME_OVER) {
        // Use game camera and dynamic fade factors
        currentOffsetX = cameraOffsetX;
        currentOffsetY = cameraOffsetY;
        currentNebulaFade = nebulaFadeFactor;
        currentMeteorFade = meteorFadeFactor;
    } else if (currentState == MAIN_MENU || currentState == OPTIONS_MENU) {
        // Use menu scroll offset and selected menu background
        currentOffsetX = menuBackgroundOffset;
        currentOffsetY = 0; // No vertical scroll in menu background
        menuBgToDraw = currentMenuBackground;

        // Set fade factors based on selected menu background
        if (menuBgToDraw == MENU_NEBULA) {
            currentNebulaFade = NEBULA_MAX_FADE; // Show nebula at max allowed fade
            currentMeteorFade = 0.0f;
        } else if (menuBgToDraw == MENU_METEOR) {
            currentNebulaFade = 0.0f;
            currentMeteorFade = 1.0f; // Show meteors fully opaque
        } else { // MENU_STARFIELD
            currentNebulaFade = 0.0f;
            currentMeteorFade = 0.0f;
        }
    }

    // Always draw stars first (they are the base layer)
    drawStars(currentOffsetX, currentOffsetY);

    // Draw Nebula if it has fade > 0
    if (currentNebulaFade > 0.0f) {
        drawNebula(currentOffsetX, currentOffsetY, currentNebulaFade);
    }

    // Draw Meteors if they have fade > 0
    if (currentMeteorFade > 0.0f) {
        drawMeteors(currentOffsetX, currentOffsetY, currentMeteorFade);
    }
}

// --- Terrain Generation ---
void generateTerrainIfNeeded() {
    float generateAheadX = player.position.x + screenWidth * TERRAIN_GENERATION_AHEAD_FACTOR;
    while (lastTerrainX < generateAheadX) {
        float heightChange = random(MIN_TERRAIN_HEIGHT_CHANGE * 100, MAX_TERRAIN_HEIGHT_CHANGE * 100) / 100.0f;
        float nextY = constrain_val(lastTerrainY + heightChange, 50.0f, screenHeight * 1.5f);
        TerrainSegment newSegment = {{lastTerrainX, lastTerrainY}, {lastTerrainX + TERRAIN_SEGMENT_LENGTH, nextY}};
        calculateSegmentVectors(newSegment); terrainSegments.push_back(newSegment);
        lastTerrainX = newSegment.end.x; lastTerrainY = newSegment.end.y;
    }
    float deleteBehindX = player.position.x - screenWidth * 2.0f;
    while (!terrainSegments.empty() && terrainSegments[0].end.x < deleteBehindX) terrainSegments.erase(terrainSegments.begin());
    while (terrainSegments.size() > MAX_TERRAIN_SEGMENTS * 1.5) { if (!terrainSegments.empty()) terrainSegments.erase(terrainSegments.begin()); else break; }
}

// --- Physics Update Function ---
void updatePhysics(float dt) {
    // Apply global forces
    player.acceleration += {0, GRAVITY / player.mass};
    if (player.wheels[0].onGround || player.wheels[1].onGround) player.acceleration -= player.velocity * (AIR_RESISTANCE_LINEAR / player.mass);
    player.angularAcceleration -= player.angularVelocity * AIR_RESISTANCE_ANGULAR / player.inertia;

    // Suspension forces
    for (int i = 0; i < 2; ++i) {
        Wheel& wheel = player.wheels[i]; Vec2D suspensionAttachWorld = getChassisPointInWorld(player, wheel.chassisAttachmentPointLocal);
        Vec2D suspensionAxisWorld = player.localSuspensionAxis[i].rotated(player.angle); Vec2D currentSuspensionVector = wheel.position - suspensionAttachWorld;
        float lengthAlongAxis = dot(currentSuspensionVector, suspensionAxisWorld); Vec2D chassisPointVelocity = getChassisVelocityAtPoint(player, suspensionAttachWorld);
        Vec2D relativeVelocity = wheel.velocity - chassisPointVelocity; float velocityAlongAxis = dot(relativeVelocity, suspensionAxisWorld);
        float springForceMag = -SUSPENSION_STIFFNESS * (lengthAlongAxis - SUSPENSION_LENGTH); float dampingForceMag = -SUSPENSION_DAMPING * velocityAlongAxis;
        float totalForceMag = constrain_val(springForceMag + dampingForceMag, -SUSPENSION_MAX_FORCE, SUSPENSION_MAX_FORCE);
        wheel.suspensionForce = suspensionAxisWorld * totalForceMag; Vec2D forceOnChassis = wheel.suspensionForce * -1.0f;
        player.acceleration += forceOnChassis / player.mass; Vec2D radiusVector = suspensionAttachWorld - player.position;
        float torque = radiusVector.x * forceOnChassis.y - radiusVector.y * forceOnChassis.x; player.angularAcceleration += torque / player.inertia;
    }

    // Integrate chassis
    player.velocity += player.acceleration * dt; player.position += player.velocity * dt;
    player.angularVelocity += player.angularAcceleration * dt; player.angularVelocity = constrain_val(player.angularVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    player.angle += player.angularVelocity * dt;

    // Integrate wheels & constraints
    for (int i = 0; i < 2; ++i) {
        Wheel& wheel = player.wheels[i]; wheel.velocity += (wheel.suspensionForce / wheel.mass) * dt; wheel.position += wheel.velocity * dt;
        Vec2D suspensionAttachWorld = getChassisPointInWorld(player, wheel.chassisAttachmentPointLocal); Vec2D suspensionAxisWorld = player.localSuspensionAxis[i].rotated(player.angle);
        Vec2D currentSuspensionVector = wheel.position - suspensionAttachWorld; float lengthAlongAxis = dot(currentSuspensionVector, suspensionAxisWorld);
        wheel.position = suspensionAttachWorld + suspensionAxisWorld * lengthAlongAxis; // Constraint
        float minLength = SUSPENSION_LENGTH * SUSPENSION_MIN_COMPRESS_FACTOR; float maxLength = SUSPENSION_LENGTH * SUSPENSION_MAX_STRETCH_FACTOR;
        bool clamped = false; float clampedLength = lengthAlongAxis;
        if (lengthAlongAxis < minLength) { clampedLength = minLength; clamped = true; } else if (lengthAlongAxis > maxLength) { clampedLength = maxLength; clamped = true; }
        if (clamped) { wheel.position = suspensionAttachWorld + suspensionAxisWorld * clampedLength; Vec2D chassisPointVelocity = getChassisVelocityAtPoint(player, suspensionAttachWorld); Vec2D relativeVel = wheel.velocity - chassisPointVelocity; float velAlongAxis = dot(relativeVel, suspensionAxisWorld); if ((clampedLength == maxLength && velAlongAxis > 0) || (clampedLength == minLength && velAlongAxis < 0)) { wheel.velocity -= suspensionAxisWorld * velAlongAxis; } }
        wheel.currentSuspensionLength = (wheel.position - suspensionAttachWorld).length();

        // Anti-Tunneling Floor
        float terrainY = getTerrainHeightAt(wheel.position.x);
        if (terrainY < screenHeight * 2.9f) { float minY = terrainY - wheel.radius; if (wheel.position.y > minY) { wheel.position.y = minY; if (wheel.velocity.y > 0) wheel.velocity.y = 0; } }

        // Visual wheel spin
        float wheelAngularStep = 0.0f; if (wheel.radius > 0.01f) { wheelAngularStep = (player.velocity.x / wheel.radius) * dt; if (player.isAccelerating) wheelAngularStep += VISUAL_ACCEL_SPIN_RATE * dt; } wheel.angle += wheelAngularStep;
        wheel.onGround = false; // Reset flag
    }

    // Boundary Check (Fall off bottom)
    if (player.position.y - cameraOffsetY > screenHeight * 1.5 && currentState == PLAYING) { currentState = GAME_OVER; menuEntryTime = millis(); scoreSaved = false; acceptGameOverInput = false; gameOverTitleAngle = 0.0f; gameOverTitleAngleDir = 1; }
}

// --- Particle Update Function ---
void updateParticles(float dt) {
    for (int i = particles.size() - 1; i >= 0; --i) {
        Particle& p = particles[i]; p.life -= dt;
        if (p.life <= 0) { particles.erase(particles.begin() + i); }
        else { p.velocity.y += PARTICLE_GRAVITY * dt; p.position += p.velocity * dt; }
    }
}

// --- Particle Spawning Function ---
void spawnParticles(const Wheel& wheel, float playerSpeed) {
    if (particles.size() >= MAX_PARTICLES) return;
    int numToSpawn = constrain_val((int)(abs(playerSpeed) * PARTICLE_COUNT_FACTOR), 0, 5);
    float baseParticleSpeed = abs(playerSpeed) * PARTICLE_SPEED_FACTOR;
    Vec2D groundTangent = {-wheel.contactNormal.y, wheel.contactNormal.x};
    if (player.velocity.x > 0 && groundTangent.x < 0) groundTangent *= -1.0f; if (player.velocity.x < 0 && groundTangent.x > 0) groundTangent *= -1.0f;
    Vec2D baseDir = (groundTangent * -1.0f + Vec2D{0, -0.3f}).normalized();
    for (int i = 0; i < numToSpawn && particles.size() < MAX_PARTICLES; ++i) {
        Particle p; p.position = wheel.contactPoint; float angleOffset = random(-PARTICLE_ANGLE_SPREAD * 100, PARTICLE_ANGLE_SPREAD * 100) / 200.0f;
        float speedMult = random(70, 130) / 100.0f; p.velocity = baseDir.rotated(angleOffset) * baseParticleSpeed * speedMult;
        p.life = PARTICLE_LIFETIME * (random(80, 120) / 100.0f); p.color = PARTICLE_COLOR; particles.push_back(p);
    }
}

// --- Collision Resolution (Wheel vs. Terrain) ---
void resolveWheelTerrainCollision(Wheel& wheel, BikeChassis& chassis, const Vec2D& collisionPoint, const Vec2D& collisionNormal, float dt) {
    wheel.onGround = true; wheel.contactPoint = collisionPoint; wheel.contactNormal = collisionNormal;
    // Penetration Correction
    float penetration = wheel.radius - dot(wheel.position - collisionPoint, collisionNormal);
    if (penetration > 0.0f) {
        Vec2D correction = collisionNormal * penetration; wheel.position += correction;
        int wheelIndex = (&wheel == &player.wheels[0]) ? 0 : 1; Vec2D suspensionAttachWorld = getChassisPointInWorld(player, player.wheels[wheelIndex].chassisAttachmentPointLocal);
        Vec2D suspensionAxisWorld = player.localSuspensionAxis[wheelIndex].rotated(player.angle); Vec2D correctedSuspVec = wheel.position - suspensionAttachWorld;
        float correctedLengthAlongAxis = dot(correctedSuspVec, suspensionAxisWorld); wheel.position = suspensionAttachWorld + suspensionAxisWorld * correctedLengthAlongAxis;
        wheel.currentSuspensionLength = correctedLengthAlongAxis;
    }
    // Velocity Response
    Vec2D relativeVelocity = wheel.velocity; float velocityAlongNormal = dot(relativeVelocity, collisionNormal);
    if (velocityAlongNormal < 0) {
        float restitution = 0.35f; float j_restitution = -(1.0f + restitution) * velocityAlongNormal;
        Vec2D restitutionImpulse = collisionNormal * j_restitution; wheel.velocity += restitutionImpulse;
        relativeVelocity = wheel.velocity; Vec2D tangent = {-collisionNormal.y, collisionNormal.x}; float velocityAlongTangent = dot(relativeVelocity, tangent);
        float frictionImpulseMag = -velocityAlongTangent; float maxFrictionImpulseMag = abs(j_restitution) * FRICTION_COEFFICIENT;
        frictionImpulseMag = constrain_val(frictionImpulseMag, -maxFrictionImpulseMag, maxFrictionImpulseMag);
        Vec2D frictionImpulse = tangent * frictionImpulseMag; wheel.velocity += frictionImpulse;
        // Ensure non-penetrating velocity after impulses
        float finalVelocityAlongNormal = dot(wheel.velocity, collisionNormal);
        if (finalVelocityAlongNormal < 0) { wheel.velocity -= collisionNormal * finalVelocityAlongNormal; }
    }
}

// --- Check Collisions ---
void checkAndResolveCollisions(float dt) {
    // Check Wheel Collisions
    for (int i = 0; i < 2; ++i) {
        Wheel& wheel = player.wheels[i];
        for (const auto& segment : terrainSegments) {
            float checkRadius = wheel.radius + 10.0f;
            if (std::max(segment.start.x, segment.end.x) >= wheel.position.x - checkRadius && std::min(segment.start.x, segment.end.x) <= wheel.position.x + checkRadius &&
                std::max(segment.start.y, segment.end.y) >= wheel.position.y - checkRadius && std::min(segment.start.y, segment.end.y) <= wheel.position.y + checkRadius * 2) { // Adjusted Y check
                Vec2D cP, cN; if (checkWheelTerrainCollision(wheel, segment, cP, cN)) { resolveWheelTerrainCollision(wheel, player, cP, cN, dt); break; } // Resolve first collision found
            }
        }
    }
     // Check Chassis Collision -> Game Over
     Vec2D chassisCheckPointsLocal[] = {{0, player.height * 0.5f - CHASSIS_COLLISION_Y_OFFSET}, {player.width * CHASSIS_COLLISION_X_FACTOR, player.height * 0.5f - CHASSIS_COLLISION_Y_OFFSET}, {-player.width * CHASSIS_COLLISION_X_FACTOR, player.height * 0.5f - CHASSIS_COLLISION_Y_OFFSET}};
     for(const auto& localPt : chassisCheckPointsLocal) {
         Vec2D checkPointWorld = getChassisPointInWorld(player, localPt);
         for (const auto& segment : terrainSegments) {
             if (std::max(segment.start.x, segment.end.x) >= checkPointWorld.x - 10.0f && std::min(segment.start.x, segment.end.x) <= checkPointWorld.x + 10.0f) {
                 Vec2D cP, cN; if (checkChassisPointTerrainCollision(checkPointWorld, segment, cP, cN)) { currentState = GAME_OVER; menuEntryTime = millis(); scoreSaved = false; acceptGameOverInput = false; gameOverTitleAngle = 0.0f; gameOverTitleAngleDir = 1; if(soundVolume > 0) M5.Speaker.tone(GAMEOVER_SOUND_FREQ, GAMEOVER_SOUND_DUR); return; }
             }
         }
     }
     // Check Rider Head Collision -> Game Over
     Vec2D riderHeadLocal = {0, RIDER_HEAD_OFFSET_Y}; Vec2D riderHeadWorld = player.position + riderHeadLocal.rotated(player.angle + player.riderLeanAngle);
     for (const auto& segment : terrainSegments) {
         if (std::max(segment.start.x, segment.end.x) >= riderHeadWorld.x - 10.0f && std::min(segment.start.x, segment.end.x) <= riderHeadWorld.x + 10.0f) {
             Vec2D cP, cN; if (checkChassisPointTerrainCollision(riderHeadWorld, segment, cP, cN)) { currentState = GAME_OVER; menuEntryTime = millis(); scoreSaved = false; acceptGameOverInput = false; gameOverTitleAngle = 0.0f; gameOverTitleAngleDir = 1; if(soundVolume > 0) M5.Speaker.tone(GAMEOVER_SOUND_FREQ, GAMEOVER_SOUND_DUR); return; }
         }
     }
}

// --- Apply Drive Force/Torque ---
void applyDriveForce(Wheel& wheel, float torqueFraction) {
    // Note: Drive force scaling based on compression removed for now
    if (wheel.radius > 0.01f) {
        float driveForceMagnitude = (DRIVE_TORQUE * torqueFraction / wheel.radius);
        Vec2D tangentDir = {-wheel.contactNormal.y, wheel.contactNormal.x}; // Direction along the ground
        Vec2D chassisForward = Vec2D{1, 0}.rotated(player.angle); // Chassis forward direction

        // Ensure drive force pushes generally forward relative to chassis
        if (dot(tangentDir, chassisForward) < 0) tangentDir *= -1.0f;
        Vec2D driveForce = tangentDir * driveForceMagnitude;

        // Apply force to chassis center of mass
        player.acceleration += driveForce / player.mass;

        // Apply torque around chassis center of mass
        Vec2D radiusVector = wheel.contactPoint - player.position;
        // If contact point is too close (or invalid), use wheel attach point for torque calculation
        if(radiusVector.length() < 0.1f) {
             radiusVector = getChassisPointInWorld(player, wheel.chassisAttachmentPointLocal) - player.position;
        }
        float groundTorque = radiusVector.x * driveForce.y - radiusVector.y * driveForce.x; // 2D cross product
        player.angularAcceleration += groundTorque / player.inertia;
    }
}

// --- Input Handling Function ---  Options Menu Updated
void handleInput(float dt) {
    unsigned long now = millis(); bool interaction = false;
    // General key check
    bool keyHeld = false; if (M5Cardputer.Keyboard.isKeyPressed(',') || M5Cardputer.Keyboard.isKeyPressed('/') || M5Cardputer.Keyboard.isKeyPressed('a') || M5Cardputer.Keyboard.isKeyPressed('d') || M5Cardputer.Keyboard.isKeyPressed(' ') || M5Cardputer.Keyboard.isKeyPressed(';') || M5Cardputer.Keyboard.isKeyPressed('.') || M5Cardputer.Keyboard.isKeyPressed('\r')|| M5Cardputer.Keyboard.isKeyPressed('`')) keyHeld = true;
    if (M5Cardputer.Keyboard.isChange() || keyHeld) { lastInteractionTime = now; interaction = true; }

    switch (currentState) {
        case MAIN_MENU: {
            bool actionTaken = false; bool navigated = false; bool selected = false;
            if (now - lastMenuNavTime > MENU_NAV_COOLDOWN) {
                if (M5Cardputer.Keyboard.isKeyPressed(';')) { // Up Arrow
                    selectedMenuItem = (selectedMenuItem - 1 + MAIN_MENU_ITEMS) % MAIN_MENU_ITEMS; actionTaken = true; navigated = true;
                } else if (M5Cardputer.Keyboard.isKeyPressed('.')) { // Down Arrow
                    selectedMenuItem = (selectedMenuItem + 1) % MAIN_MENU_ITEMS; actionTaken = true; navigated = true;
                } else if (M5Cardputer.Keyboard.isKeyPressed('\r') || M5Cardputer.Keyboard.isKeyPressed(' ')) { // Select
                    if (selectedMenuItem == 0) { // Play
                        resetGame(); currentState = PLAYING;
                    } else if (selectedMenuItem == 1) { // Options
                         currentState = OPTIONS_MENU;
                         selectedOptionItem = 0; // Reset selection
                         optionsMenuScrollOffset = 0; // Reset scroll
                         menuEntryTime = now;
                         lastMenuNavTime = now; lastMenuAdjustTime = now;
                    }
                    actionTaken = true; selected = true;
                }
            }
            if (actionTaken) {
                lastMenuNavTime = now;
                if (navigated && soundVolume > 0) M5.Speaker.tone(MENU_NAV_SOUND_FREQ, MENU_NAV_SOUND_DUR);
                else if (selected && soundVolume > 0) M5.Speaker.tone(MENU_SEL_SOUND_FREQ, MENU_SEL_SOUND_DUR);
            }
        } break;

        case OPTIONS_MENU: {
             bool navigated = false; bool selected = false; bool adjusted = false; bool scrollChanged = false;

             // Back Key (Esc/Backtick)
             if (M5Cardputer.Keyboard.isKeyPressed('`') && (now - lastMenuNavTime > MENU_NAV_COOLDOWN)) {
                 currentState = MAIN_MENU;
                 menuEntryTime = now;
                 lastMenuNavTime = now;
                 selected = true; // Use selection sound for back
             }

             // Value Adjustment Keys (< , > /)
             bool adjustKeyPressed = M5Cardputer.Keyboard.isKeyPressed(',') || M5Cardputer.Keyboard.isKeyPressed('/');
             if (adjustKeyPressed && (now - lastMenuAdjustTime > MENU_ADJUST_COOLDOWN)) {
                 bool valueChanged = false; float adjFactor = M5Cardputer.Keyboard.isKeyPressed(',') ? -1.0f : 1.0f;
                 preferences.begin("sunrider", false); // Open for writing
                 switch(selectedOptionItem) { // Adjust the currently selected item
                     case 0: brightnessOption = constrain_val(brightnessOption + adjFactor * BRIGHTNESS_STEP, 0.0f, 100.0f); M5Cardputer.Display.setBrightness((uint8_t)constrain_val(brightnessOption * 2.55f, 0.0f, 255.0f)); preferences.putFloat("bright", brightnessOption); valueChanged = true; break;
                     case 1: soundVolume = constrain_val(soundVolume + adjFactor * VOLUME_STEP, 0.0f, 100.0f); M5.Speaker.setVolume((uint8_t)mapfloat(soundVolume, 0.0f, 100.0f, 0.0f, 255.0f)); preferences.putFloat("volume", soundVolume); valueChanged = true; break;
                     case 2: GRAVITY = constrain_val(GRAVITY + adjFactor * GRAVITY_STEP, MIN_GRAVITY, MAX_GRAVITY); preferences.putFloat("gravity", GRAVITY); valueChanged = true; break;
                     case 3: DRIVE_TORQUE = constrain_val(DRIVE_TORQUE + adjFactor * TORQUE_STEP, MIN_TORQUE, MAX_TORQUE); preferences.putFloat("torque", DRIVE_TORQUE); valueChanged = true; break;
                     case 4: infoCornerPosition = (InfoCorner)(((int)infoCornerPosition + (adjFactor > 0 ? 1 : -1) + (int)INFO_CORNER_COUNT) % (int)INFO_CORNER_COUNT); preferences.putInt("infocorn", (int)infoCornerPosition); valueChanged = true; break;
                     case 5: smoothCameraEnabled = !smoothCameraEnabled; preferences.putBool("smoothcam", smoothCameraEnabled); valueChanged = true; break;
                     case 6: currentTerrainColorIndex = (currentTerrainColorIndex + 1) % NUM_TERRAIN_COLORS; preferences.putInt("terraincolor", currentTerrainColorIndex); valueChanged = true; break; // Cycle forward only
                     case 7: currentMenuBackground = (MenuBackground)(((int)currentMenuBackground + 1) % (int)MENU_BACKGROUND_COUNT); preferences.putInt("menubg", (int)currentMenuBackground); valueChanged = true; break; //  Cycle menu BG forward only
                 }
                 preferences.end(); // Save changes
                 if (valueChanged) { lastMenuAdjustTime = now; adjusted = true; }
             }

             // Navigation Keys (Up ; Down .) Scrolling Logic
             bool navActionTaken = false;
             if (now - lastMenuNavTime > MENU_NAV_COOLDOWN) {
                 if (M5Cardputer.Keyboard.isKeyPressed(';')) { // Up Arrow
                     int oldSelected = selectedOptionItem;
                     selectedOptionItem = (selectedOptionItem - 1 + OPTIONS_MENU_ITEMS) % OPTIONS_MENU_ITEMS;
                     // Check if scroll needed
                     if (selectedOptionItem < optionsMenuScrollOffset) {
                         optionsMenuScrollOffset = selectedOptionItem;
                         scrollChanged = true;
                     } else if (oldSelected == optionsMenuScrollOffset && optionsMenuScrollOffset > 0 && selectedOptionItem == OPTIONS_MENU_ITEMS - 1) {
                         // Handle wrapping selection from top visible item when scrolled down (going up to last item)
                          optionsMenuScrollOffset = std::max(0, OPTIONS_MENU_ITEMS - MAX_OPTIONS_VISIBLE);
                          scrollChanged = true;
                     }
                     // Handle wrap around from first item to last visually
                     if (oldSelected == 0 && selectedOptionItem == OPTIONS_MENU_ITEMS - 1) {
                         optionsMenuScrollOffset = std::max(0, OPTIONS_MENU_ITEMS - MAX_OPTIONS_VISIBLE);
                         scrollChanged = true;
                     }

                     navActionTaken = true; navigated = true;
                 } else if (M5Cardputer.Keyboard.isKeyPressed('.')) { // Down Arrow
                     int oldSelected = selectedOptionItem;
                     selectedOptionItem = (selectedOptionItem + 1) % OPTIONS_MENU_ITEMS;
                     // Check if scroll needed
                     int bottomVisibleIndex = optionsMenuScrollOffset + MAX_OPTIONS_VISIBLE - 1;
                     if (selectedOptionItem > bottomVisibleIndex) {
                          optionsMenuScrollOffset = selectedOptionItem - MAX_OPTIONS_VISIBLE + 1;
                          scrollChanged = true;
                     } else if (oldSelected == bottomVisibleIndex && optionsMenuScrollOffset < OPTIONS_MENU_ITEMS - MAX_OPTIONS_VISIBLE && selectedOptionItem == 0) {
                          // Handle wrapping selection from bottom visible item when scrolled up (going down to first item)
                          optionsMenuScrollOffset = 0;
                          scrollChanged = true;
                     }
                     // Handle wrap around from last item to first visually
                     if (oldSelected == OPTIONS_MENU_ITEMS - 1 && selectedOptionItem == 0) {
                         optionsMenuScrollOffset = 0;
                         scrollChanged = true;
                     }
                     navActionTaken = true; navigated = true;
                 }
             }

             // Clamp scroll offset just in case
             optionsMenuScrollOffset = constrain_val(optionsMenuScrollOffset, 0, std::max(0, OPTIONS_MENU_ITEMS - MAX_OPTIONS_VISIBLE));

             // Play Sounds
             if (navActionTaken || adjusted || selected) {
                 lastMenuNavTime = now; // Reset cooldown for any action
                 if (navigated && soundVolume > 0) M5.Speaker.tone(MENU_NAV_SOUND_FREQ, MENU_NAV_SOUND_DUR);
                 else if (selected && soundVolume > 0) M5.Speaker.tone(MENU_SEL_SOUND_FREQ, MENU_SEL_SOUND_DUR); // Back uses select sound
                 else if (adjusted && soundVolume > 0) M5.Speaker.tone(MENU_NAV_SOUND_FREQ, MENU_NAV_SOUND_DUR); // Adjust uses nav sound
             }
        } break;

        case PLAYING: {
            player.isAccelerating = M5Cardputer.Keyboard.isKeyPressed(' '); bool leaning = false; float targetRiderLean = 0.0f;
            if (M5Cardputer.Keyboard.isKeyPressed('a')) { player.angularAcceleration -= LEAN_TORQUE / player.inertia; leaning = true; targetRiderLean = -RIDER_LEAN_ANGLE_DEG * M_PI / 180.0f; }
            if (M5Cardputer.Keyboard.isKeyPressed('d')) { player.angularAcceleration += LEAN_TORQUE / player.inertia; leaning = true; targetRiderLean = RIDER_LEAN_ANGLE_DEG * M_PI / 180.0f; }
            player.riderLeanAngle += (targetRiderLean - player.riderLeanAngle) * 0.15f; // Smooth lean transition
            if (player.isAccelerating) { bool rearGrounded = player.wheels[0].onGround; bool frontGrounded = player.wheels[1].onGround; float currentSpeed = player.velocity.length(); if (rearGrounded || frontGrounded) { if (rearGrounded) { applyDriveForce(player.wheels[0], frontGrounded ? 0.5f : 1.0f); spawnParticles(player.wheels[0], currentSpeed); } if (frontGrounded) { applyDriveForce(player.wheels[1], rearGrounded ? 0.5f : 1.0f); spawnParticles(player.wheels[1], currentSpeed); } if (soundVolume > 0) { float speedFactor = constrain_val(currentSpeed / MAX_SPEED_FOR_SOUND, 0.0f, 1.0f); float intervalFactor = 1.0f - sqrt(speedFactor); unsigned long currentSoundInterval = constrain_val((unsigned long)(MIN_SOUND_INTERVAL + (MAX_SOUND_INTERVAL - MIN_SOUND_INTERVAL) * intervalFactor), MIN_SOUND_INTERVAL, MAX_SOUND_INTERVAL); if (now - lastSoundPlayTime > currentSoundInterval) { M5.Speaker.tone(ENGINE_NOTE_FREQ, ENGINE_NOTE_DURATION); lastSoundPlayTime = now; } } } }
            // Auto-balancing when grounded and not leaning
            if ((player.wheels[0].onGround || player.wheels[1].onGround) && !leaning) { float targetAngle = 0; float correctionFactor = 0.3f; float correctionTorque = constrain_val((targetAngle - player.angle) * player.inertia * correctionFactor, -LEAN_TORQUE * 0.5f, LEAN_TORQUE * 0.5f); player.angularAcceleration += correctionTorque / player.inertia; player.angularVelocity *= pow(0.75, dt * 60.0f); } // Dampen angular velocity slightly when auto-balancing
        } break;
        case GAME_OVER: {
            if (!scoreSaved) { if (totalDistanceTravelled > highScore) { highScore = totalDistanceTravelled; preferences.begin("sunrider", false); preferences.putFloat("highscore", highScore); preferences.end(); if(soundVolume > 0) { M5.Speaker.tone(HIGHSCORE_SOUND_FREQ1, HIGHSCORE_SOUND_DUR1); delay(HIGHSCORE_SOUND_DELAY); M5.Speaker.tone(HIGHSCORE_SOUND_FREQ2, HIGHSCORE_SOUND_DUR2); } } scoreSaved = true; }
            if (!M5Cardputer.Keyboard.isKeyPressed(' ')) acceptGameOverInput = true; // Wait for space release
            bool actionTaken = false;
            if (acceptGameOverInput && M5Cardputer.Keyboard.isKeyPressed(' ') && (now - lastMenuNavTime > MENU_NAV_COOLDOWN * 2)) { currentState = MAIN_MENU; menuEntryTime = now; actionTaken = true; if (soundVolume > 0) M5.Speaker.tone(MENU_SEL_SOUND_FREQ, MENU_SEL_SOUND_DUR); }
            if (actionTaken) lastMenuNavTime = now;
        } break;
    }
}

// --- Rendering Function ---  Options Menu Updated
// Draws the current game state to the canvas buffer, then pushes it to the display.
void render() {
    canvas.fillSprite(BLACK); // Clear the drawing buffer
    drawBackground();       // Draw stars, nebula, meteors based on state/settings

    switch (currentState) {
        case MAIN_MENU: {
            // --- Render Main Menu ---
            unsigned long now = millis(); unsigned long timeSinceEntry = now - menuEntryTime; float fade = constrain_val((float)timeSinceEntry / MENU_FADE_IN_MS, 0.0f, 1.0f);
            float titleX = screenWidth / 2; float titleY = screenHeight / 4; const char* titleText = "Sun Rider";
            // Rotating Background Title
            canvas.setTextDatum(middle_center); canvas.setTextSize(2); int bgTextWidth = canvas.textWidth(titleText); int bgTextHeight = 8 * 2; int spriteSize = std::max((int)(bgTextWidth * 1.2), (int)(bgTextHeight * 1.2));
            titleSprite.createSprite(spriteSize, spriteSize); titleSprite.fillSprite(BLACK); titleSprite.setTextDatum(middle_center); titleSprite.setTextColor(RED); titleSprite.setTextSize(2); titleSprite.drawString(titleText, spriteSize / 2, spriteSize / 2); titleSprite.setPivot(spriteSize / 2.0f, spriteSize / 2.0f); titleSprite.pushRotateZoomWithAA(titleX, titleY, menuTitleAngle, 1.0f, 1.0f, BLACK); titleSprite.deleteSprite();
            // Foreground Title
            canvas.setTextDatum(middle_center); canvas.setTextColor(fadeColor(YELLOW, BLACK, fade)); canvas.setTextSize(2); canvas.drawString(titleText, titleX, titleY);
            // High Score
            canvas.setTextSize(1); canvas.setTextColor(fadeColor(CYAN, BLACK, fade)); canvas.drawString("High Score: " + String(highScore / DISTANCE_SCALE_FACTOR, 1) + "m", screenWidth / 2, screenHeight / 4 + 25);
            // Menu Items
            canvas.setTextSize(1); int itemY = screenHeight / 2 + 10; int itemHeight = 15; int fontHeight = 8 * 1; uint16_t normalItemColor = fadeColor(WHITE, BLACK, fade); uint16_t selectedItemColorFg = fadeColor(BLACK, BLACK, fade); uint16_t selectedItemColorBg = fadeColor(WHITE, BLACK, fade);
            const char* playText = "Play"; int playTextWidth = canvas.textWidth(playText); if (selectedMenuItem == 0) { canvas.fillRect(titleX - playTextWidth/2 - MENU_SELECTION_PADDING, itemY - fontHeight/2 - MENU_SELECTION_PADDING, playTextWidth + 2 * MENU_SELECTION_PADDING, fontHeight + 2 * MENU_SELECTION_PADDING, selectedItemColorBg); canvas.setTextColor(selectedItemColorFg); } else { canvas.setTextColor(normalItemColor); } canvas.drawString(playText, titleX, itemY);
            itemY += itemHeight; const char* optionsText = "Options"; int optionsTextWidth = canvas.textWidth(optionsText); if (selectedMenuItem == 1) { canvas.fillRect(titleX - optionsTextWidth/2 - MENU_SELECTION_PADDING, itemY - fontHeight/2 - MENU_SELECTION_PADDING, optionsTextWidth + 2 * MENU_SELECTION_PADDING, fontHeight + 2 * MENU_SELECTION_PADDING, selectedItemColorBg); canvas.setTextColor(selectedItemColorFg); } else { canvas.setTextColor(normalItemColor); } canvas.drawString(optionsText, titleX, itemY);
            // Menu Hints
            uint16_t hintColor = fadeColor(DARKGREY, BLACK, fade); canvas.setTextColor(hintColor); canvas.setTextDatum(bottom_center);
            canvas.drawString("NAV: Arrows  SEL: Space  BACK: Esc", screenWidth / 2, screenHeight - 5);
            canvas.setTextDatum(top_left);
        } break;

        case OPTIONS_MENU: {
            // --- Render Options Menu ---
            unsigned long now = millis(); unsigned long timeSinceEntry = now - menuEntryTime; float fade = constrain_val((float)timeSinceEntry / MENU_FADE_IN_MS, 0.0f, 1.0f);
            canvas.setTextDatum(middle_center); canvas.setTextColor(fadeColor(YELLOW, BLACK, fade)); canvas.setTextSize(1); canvas.drawString("Options", screenWidth / 2, 15);

            int itemY = 30; // Start Y position for items
            int itemHeight = 12; // Height between items
            int valueX = screenWidth / 2 + 10;
            int labelX = 10; // Arrow horizontal position
            int fontHeight = 8 * 1;
            uint16_t normalItemColor = fadeColor(WHITE, BLACK, fade);
            uint16_t selectedItemColorFg = fadeColor(BLACK, BLACK, fade);
            uint16_t selectedItemColorBg = fadeColor(WHITE, BLACK, fade);
            uint16_t arrowColor = fadeColor(LIGHTGREY, BLACK, fade);

            canvas.setTextSize(1);
            // Draw Up Arrow if needed
            if (optionsMenuScrollOffset > 0) {
                canvas.setTextColor(arrowColor);
                canvas.setTextDatum(middle_center);
                canvas.drawString("^", labelX, itemY - itemHeight/2 - 2 + 5); // Align with labelX, move down 5px
            }

            // Draw visible options
            canvas.setTextDatum(top_left); // Reset datum for labels/values
            for (int i = 0; i < MAX_OPTIONS_VISIBLE; ++i) {
                 int actualIndex = optionsMenuScrollOffset + i;
                 if (actualIndex >= OPTIONS_MENU_ITEMS) break; // Don't draw past the end

                 int currentY = itemY + i * itemHeight;
                 const char* label = "";
                 String valueStr = "";

                 switch(actualIndex) {
                     case 0: label = "Brightness:"; valueStr = "< " + String(brightnessOption, 0) + " % >"; break;
                     case 1: label = "Sound Volume:"; valueStr = "< " + String(soundVolume, 0) + " % >"; break;
                     case 2: label = "Gravity:"; valueStr = "< " + String(GRAVITY, 1) + " >"; break;
                     case 3: label = "Drive Torque:"; valueStr = "< " + String(DRIVE_TORQUE, 0) + " >"; break;
                     case 4: label = "Info Corner:"; valueStr = "< " + String(getInfoCornerName(infoCornerPosition)) + " >"; break;
                     case 5: label = "Smooth Camera:"; valueStr = "< " + String(getOnOffName(smoothCameraEnabled)) + " >"; break;
                     case 6: label = "Terrain Color:"; valueStr = "< " + String(getTerrainColorName(currentTerrainColorIndex)) + " >"; break;
                     case 7: label = "Menu BG:"; valueStr = "< " + String(getMenuBackgroundName(currentMenuBackground)) + " >"; break; // 
                     default: label = "???"; valueStr = "???"; break;
                 }

                 int labelWidth = canvas.textWidth(label);
                 int valueWidth = canvas.textWidth(valueStr.c_str());
                 int highlightWidth = screenWidth - (labelX - MENU_SELECTION_PADDING) * 2 ;
                 int highlightHeight = fontHeight + 2 * MENU_SELECTION_PADDING;
                 int highlightX = labelX - MENU_SELECTION_PADDING;
                 int highlightY = currentY - MENU_SELECTION_PADDING;

                 if (actualIndex == selectedOptionItem) {
                     canvas.fillRect(highlightX, highlightY, highlightWidth, highlightHeight, selectedItemColorBg);
                     canvas.setTextColor(selectedItemColorFg);
                 } else {
                     canvas.setTextColor(normalItemColor);
                 }
                 canvas.drawString(label, labelX, currentY);
                 canvas.drawString(valueStr.c_str(), valueX, currentY);
            }

            // Draw Down Arrow if needed
            if (optionsMenuScrollOffset + MAX_OPTIONS_VISIBLE < OPTIONS_MENU_ITEMS) {
                canvas.setTextColor(arrowColor);
                canvas.setTextDatum(middle_center);
                int arrowY = itemY + MAX_OPTIONS_VISIBLE * itemHeight - itemHeight/2 + 2 + 5; // Move down 5px
                canvas.drawString("v", labelX, arrowY); // Align with labelX
            }

            // Menu Hints
            uint16_t hintColor = fadeColor(DARKGREY, BLACK, fade);
            canvas.setTextColor(hintColor);
            canvas.setTextDatum(bottom_center);
            canvas.drawString("NAV: Arrows  SEL: Space  BACK: Esc", screenWidth / 2, screenHeight - 5); // Updated legend
            canvas.setTextDatum(top_left); // Reset datum
        } break;

        case PLAYING:
        case GAME_OVER: // Render gameplay elements for both states
        {
            // --- Draw Terrain ---
            uint16_t currentFillColor = terrainColors[currentTerrainColorIndex];
            for (const auto& segment : terrainSegments) {
                float sX = segment.start.x - cameraOffsetX; float sY = segment.start.y - cameraOffsetY;
                float eX = segment.end.x - cameraOffsetX; float eY = segment.end.y - cameraOffsetY;
                // Basic horizontal culling
                if (std::max(sX, eX) >= -10 && std::min(sX, eX) <= screenWidth + 10) {
                    // Filled style
                     if (std::max(sY, eY) >= -10) { // Basic vertical culling
                         float bottomY = screenHeight + 10;
                         // Fill below the top line
                         canvas.fillTriangle(sX, sY, eX, eY, eX, bottomY, currentFillColor);
                         canvas.fillTriangle(sX, sY, eX, bottomY, sX, bottomY, currentFillColor);
                         // Draw top line
                         canvas.drawLine(sX, sY, eX, eY, TERRAIN_SURFACE_COLOR);
                     }
                }
            }

            // --- Draw Checkpoint Flags ---
            int startCpIndex = floor((cameraOffsetX - screenWidth) / CHECKPOINT_INTERVAL); int endCpIndex = ceil((cameraOffsetX + screenWidth) / CHECKPOINT_INTERVAL);
            canvas.setTextSize(1); canvas.setTextColor(RED);
            for (int cpIdx = startCpIndex; cpIdx <= endCpIndex; ++cpIdx) {
                if (cpIdx < 0) continue; float cpX_world = (float)(cpIdx + 1) * CHECKPOINT_INTERVAL; float cpY_world = getTerrainHeightAt(cpX_world);
                if (cpY_world < screenHeight * 2.9f) { float cpX_screen = cpX_world - cameraOffsetX; float cpY_screen = cpY_world - cameraOffsetY;
                    if (cpX_screen >= -10 && cpX_screen <= screenWidth + 10) { canvas.drawLine(cpX_screen, cpY_screen, cpX_screen, cpY_screen - CHECKPOINT_FLAG_HEIGHT, RED); canvas.fillTriangle(cpX_screen, cpY_screen - CHECKPOINT_FLAG_HEIGHT, cpX_screen + CHECKPOINT_FLAG_WIDTH, cpY_screen - CHECKPOINT_FLAG_HEIGHT + (CHECKPOINT_FLAG_WIDTH / 2), cpX_screen, cpY_screen - CHECKPOINT_FLAG_HEIGHT + CHECKPOINT_FLAG_WIDTH, RED); canvas.setTextDatum(bottom_center); canvas.drawString(String((int)(((cpIdx + 1) * CHECKPOINT_INTERVAL) / DISTANCE_SCALE_FACTOR)) + "m", cpX_screen, cpY_screen - CHECKPOINT_TEXT_Y_OFFSET); }
                }
            } canvas.setTextDatum(top_left);

            // --- Draw UFO Sprite ---
            if (player.spriteData != nullptr) { float dX = player.position.x - cameraOffsetX; float dY = player.position.y - cameraOffsetY; float aD = player.angle * 180.0f / M_PI; canvas.pushImageRotateZoomWithAA( dX, dY, player.width / 2.0f, player.height / 2.0f, aD, 1.0f, 1.0f, (int)player.width, (int)player.height, player.spriteData, BLACK ); }

            // --- Draw Rider Sprite ---
            { Vec2D riderLocalPos = {0, -player.height * 0.5f - SPRITE_RIDER_HEIGHT * 0.45f}; Vec2D riderWorldPos = getChassisPointInWorld(player, riderLocalPos); float riderScreenX = riderWorldPos.x - cameraOffsetX; float riderScreenY = riderWorldPos.y - cameraOffsetY; float totalRiderAngleDeg = (player.angle + player.riderLeanAngle) * 180.0f / M_PI; canvas.pushImageRotateZoomWithAA( riderScreenX, riderScreenY, SPRITE_RIDER_WIDTH / 2.0f, SPRITE_RIDER_HEIGHT / 2.0f, totalRiderAngleDeg, 1.0f, 1.0f, SPRITE_RIDER_WIDTH, SPRITE_RIDER_HEIGHT, sprite_rider_data, BLACK ); }

            // --- Draw Wheels and Suspension Lines ---
            for (int i = 0; i < 2; ++i) { Wheel& wheel = player.wheels[i]; float wX = wheel.position.x - cameraOffsetX; float wY = wheel.position.y - cameraOffsetY; canvas.fillCircle(wX, wY, wheel.radius, TIRE_COLOR); canvas.fillCircle(wX, wY, wheel.radius - TIRE_THICKNESS, DARKGREY); int numSpokes = 4; float spokeRadius = wheel.radius - TIRE_THICKNESS * 0.5f; for(int j = 0; j < numSpokes; ++j) { float angle = wheel.angle + (j * M_PI / numSpokes); float sx = wX - cos(angle) * spokeRadius; float sy = wY - sin(angle) * spokeRadius; float ex = wX + cos(angle) * spokeRadius; float ey = wY + sin(angle) * spokeRadius; canvas.drawLine(sx, sy, ex, ey, SPOKE_COLOR); } Vec2D suspensionAttachWorld = getChassisPointInWorld(player, wheel.chassisAttachmentPointLocal); float sAX = suspensionAttachWorld.x - cameraOffsetX; float sAY = suspensionAttachWorld.y - cameraOffsetY; canvas.drawLine(sAX, sAY, wX, wY, LIGHTGREY); }

            // --- Draw Particles ---
            for (const auto& p : particles) { float pX = p.position.x - cameraOffsetX; float pY = p.position.y - cameraOffsetY; if (pX >= 0 && pX < screenWidth && pY >= 0 && pY < screenHeight) { canvas.drawPixel(pX, pY, p.color); } }

            // --- Draw UI Text ---
            if (infoCornerPosition != INFO_OFF) { canvas.setTextSize(1); int infoX = 5; int infoY = 5; int lineHeight = 10; textdatum_t datum = TL_DATUM; switch(infoCornerPosition) { case INFO_TOP_LEFT: infoX = 5; infoY = 5; datum = TL_DATUM; break; case INFO_TOP_RIGHT: infoX = screenWidth - 5; infoY = 5; datum = TR_DATUM; break; case INFO_BOTTOM_LEFT: infoX = 5; infoY = screenHeight - 5; datum = BL_DATUM; break; case INFO_BOTTOM_RIGHT: infoX = screenWidth - 5; infoY = screenHeight - 5; datum = BR_DATUM; break; default: infoX = 5; infoY = 5; datum = TL_DATUM; break; } canvas.setTextDatum(datum); int yPos1 = infoY; int yPos2 = (datum == BL_DATUM || datum == BR_DATUM) ? infoY - lineHeight : infoY + lineHeight; int yPos3 = (datum == BL_DATUM || datum == BR_DATUM) ? infoY - 2 * lineHeight : infoY + 2 * lineHeight; int yPos4 = (datum == BL_DATUM || datum == BR_DATUM) ? infoY - 3 * lineHeight : infoY + 3 * lineHeight; canvas.setTextColor(WHITE); String fpsStr = (deltaTime > 0) ? ("FPS: " + String(1.0f / deltaTime, 1)) : "FPS: ---"; canvas.drawString(fpsStr, infoX, yPos1); String distStr = "Dist: " + String(totalDistanceTravelled / DISTANCE_SCALE_FACTOR, 1) + "m"; canvas.drawString(distStr, infoX, yPos2); if (currentState == PLAYING) { canvas.setTextColor(DARKGREY); canvas.drawString("Lean: A/D", infoX, yPos3); canvas.drawString("Gas: Space", infoX, yPos4); } canvas.setTextDatum(top_left); }

            // --- Game Over Message ---
            if (currentState == GAME_OVER) { float centerX = screenWidth / 2; float centerY = screenHeight / 2; const char* gameOverText = "GAME OVER"; canvas.setTextDatum(middle_center); canvas.setTextSize(2); int bgTextWidth = canvas.textWidth(gameOverText); int bgTextHeight = 8 * 2; int spriteSize = std::max((int)(bgTextWidth * 1.2), (int)(bgTextHeight * 1.2)); gameOverTitleSprite.createSprite(spriteSize, spriteSize); gameOverTitleSprite.fillSprite(BLACK); gameOverTitleSprite.setTextDatum(middle_center); gameOverTitleSprite.setTextColor(RED); gameOverTitleSprite.setTextSize(2); gameOverTitleSprite.drawString(gameOverText, spriteSize / 2, spriteSize / 2); gameOverTitleSprite.setPivot(spriteSize / 2.0f, spriteSize / 2.0f); gameOverTitleSprite.pushRotateZoomWithAA(centerX, centerY - 25, gameOverTitleAngle, 1.0f, 1.0f, BLACK); gameOverTitleSprite.deleteSprite(); canvas.setTextSize(2); canvas.setTextColor(WHITE); canvas.setTextDatum(middle_center); canvas.drawString(gameOverText, centerX, centerY - 25); canvas.setTextSize(1); canvas.setTextColor(GREEN); canvas.drawString("Distance: " + String(totalDistanceTravelled / DISTANCE_SCALE_FACTOR, 1) + "m", centerX, centerY); canvas.setTextColor(CYAN); canvas.drawString("High Score: " + String(highScore / DISTANCE_SCALE_FACTOR, 1) + "m", centerX, centerY + 12); canvas.setTextColor(WHITE); canvas.drawString("Space for Menu", centerX, centerY + 30); canvas.setTextDatum(top_left); }
        } break; // End PLAYING/GAME_OVER case
    } // End switch(currentState)

    canvas.pushSprite(0, 0); // Push the completed frame to the physical display
}
