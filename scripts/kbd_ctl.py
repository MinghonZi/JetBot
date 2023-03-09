import keyboard

import jetbot

keyboard.add_hotkey("j", jetbot.rectilinear, (0.5,))
keyboard.add_hotkey("k", jetbot.rectilinear, (-0.5,))
keyboard.add_hotkey("h", jetbot.rotate, (-0.5,))
keyboard.add_hotkey("l", jetbot.rotate, (0.5,))
keyboard.on_release(lambda e: jetbot.brake())

keyboard.wait("esc")
