import sdl2
import asyncio

trigger = False

async def deadman_monitor():
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

def run_deadman_monitor():
    loop = asyncio.get_event_loop()
    loop.create_task(deadman_monitor())