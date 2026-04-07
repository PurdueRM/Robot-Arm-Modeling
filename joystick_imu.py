import sdl2
import sdl2.ext
from ctypes import c_float

sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO | sdl2.SDL_INIT_GAMECONTROLLER | sdl2.SDL_INIT_SENSOR)

controller = sdl2.SDL_GameControllerOpen(0)

# enable sensors
sdl2.SDL_GameControllerSetSensorEnabled(controller, sdl2.SDL_SENSOR_GYRO, True)
sdl2.SDL_GameControllerSetSensorEnabled(controller, sdl2.SDL_SENSOR_ACCEL, True)

gyro_data = (c_float * 3)()
accel_data = (c_float * 3)()

if sdl2.SDL_GameControllerHasSensor(controller, sdl2.SDL_SENSOR_GYRO):
    print("Gyro sensor is available")
    print(sdl2.SDL_GameControllerHasSensor(controller, sdl2.SDL_SENSOR_GYRO))
else:
    print("Gyro sensor is not available")

input("Press Enter to start reading sensor data...")

while True:
    sdl2.SDL_PumpEvents()  # update sensor data

    sdl2.SDL_GameControllerGetSensorData(controller, sdl2.SDL_SENSOR_GYRO, gyro_data, 3)
    sdl2.SDL_GameControllerGetSensorData(controller, sdl2.SDL_SENSOR_ACCEL, accel_data, 3)

    print("Gyro:", list(gyro_data))
    print("Accel:", list(accel_data))