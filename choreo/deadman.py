import asyncio
import time

trigger = False

async def monitor_gamepad():
    import sdl2
    global trigger
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
    sdl2.SDL_JoystickEventState(sdl2.SDL_ENABLE)
    njoy = sdl2.SDL_NumJoysticks()
    print(f"Found {njoy} joystick(s)")

    if njoy == 0:
        print("No joystick found")
        exit(1)

    joy = sdl2.SDL_JoystickOpen(0)

    event = sdl2.SDL_Event()
    while True:
        trigger_now = trigger 
        while sdl2.SDL_PollEvent(event) != 0:
            if event.type == sdl2.SDL_JOYBUTTONDOWN:
                trigger_now = True
            elif event.type == sdl2.SDL_JOYBUTTONUP:
                trigger_now = False
                break
        if trigger != trigger_now:
            trigger = trigger_now
            if trigger:
                print("Deadman trigger pressed")
            else:
                print("Deadman trigger released")
        await asyncio.sleep(0.01)

async def monitor_foot_pedal():
    from evdev import InputDevice, ecodes
    global trigger


    dev = InputDevice('/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd')
    dev.grab() 

    while True:
        keys = dev.active_keys(verbose=False)
        KEY = 48 # for whatever reason it sends the 'b' key
        now_pressed = KEY in keys
        if now_pressed and not trigger:
            print("↓  pressed")
        elif not now_pressed and trigger:
            print("↑  released")

        trigger = now_pressed


        await asyncio.sleep(0.001)

async def monitor(type="gamepad"):
    if type == "gamepad":
        await monitor_gamepad()
    elif type == "foot-pedal":
        await monitor_foot_pedal()

def run_deadman_monitor(type="gamepad"):
    loop = asyncio.get_event_loop()
    loop.create_task(monitor(type))


if __name__ == "__main__":
    asyncio.run(monitor(type="foot-pedal"))