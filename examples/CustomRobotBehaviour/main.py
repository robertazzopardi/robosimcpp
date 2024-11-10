import ctypes

DLL_PATH = "../../build/librobosim.dylib"
CONFIG_PATH = "./defaultConfig.txt".encode("utf-8")

robosim = ctypes.CDLL(DLL_PATH)

class Colour(ctypes.Structure):
    _fields_ = [('r', ctypes.c_uint8),
                ('g', ctypes.c_uint8),
                ('b', ctypes.c_uint8),
                ('a', ctypes.c_uint8)]

    def __init__(self, r, g, b, a):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

robosim.EnvController_new.argtypes = [ctypes.c_char_p]
robosim.EnvController_new.restype = ctypes.c_void_p
robosim.EnvController_run.argtypes = [ctypes.c_void_p]
robosim.EnvController_delete.argtypes = [ctypes.c_void_p]

ROBOT_RUN_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
robosim.EnvController_makeRobotsWithFunc.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.POINTER(Colour), ROBOT_RUN_FUNC]

def run_func(robot):
    print(f"Starting robot: {robot}")

if __name__ == "__main__":
    env = robosim.EnvController_new(CONFIG_PATH)

    run_func = ROBOT_RUN_FUNC(run_func)

    colour = Colour(48, 48, 48, 255)

    robosim.EnvController_makeRobotsWithFunc(env, 3, 50, colour, run_func)

    robosim.EnvController_run(env)

    robosim.EnvController_delete(env)
