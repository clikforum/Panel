"""
****************************************************************************************************************
Class HwPanel
Class to encapsulate hardware panel
****************************************************************************************************************
"""

class HwPanel():
    """
    Singleton class - ensures only one instance is ever created.
    """
    _instance = None
    ExitingApp = False
    active = ''
    hwpanel_after = None
    hwpanel_1_i2c_add = 0x20
    #hwpanel_2_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_3_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_4_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_5_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_6_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_7_i2c_add = 0xXX  #To be defined - NON Eliminare
    #hwpanel_8_i2c_add = 0xXX  #To be defined - NON Eliminare


    # Panel 'buttons'
    HWPANEL_REGISTER = 1
    HWPANEL_START_STOP  = 2
    HWPANEL_FORWARD = 3
    HWPANEL_BACKWARD = 4
    HWPANEL_FF = 5
    HWPANEL_RW = 6
    HWPANEL_FOCUS_VIEW = 7
    HWPANEL_ZOOM_VIEW = 8
    HWPANEL_ZOOM_VIEW_PLUS = 9
    HWPANEL_ZOOM_VIEW_MINUS = 10
    HWPANEL_ZOOM_VIEW_RIGHT = 11
    HWPANEL_ZOOM_VIEW_LEFT = 12
    HWPANEL_ZOOM_VIEW_UP = 13
    HWPANEL_ZOOM_VIEW_DOWN = 14
    HWPANEL_AUTO_EXPOSURE = 15
    HWPANEL_AUTO_WB = 16
    HWPANEL_AUTOSTOP_ENABLE = 17
    HWPANEL_GET_AUTOSTOP_TIME = 18
    HWPANEL_SET_AUTOSTOP_FRAMES = 19
    HWPANEL_GET_FILM_TIME = 20
    HWPANEL_GET_FPS = 21
    HWPANEL_SET_FILM_S8 = 22
    HWPANEL_SET_FILM_R8 = 23
    HWPANEL_SET_EXPOSURE = 24
    HWPANEL_SET_WB_RED = 25
    HWPANEL_SET_WB_BLUE = 26

    rpi_after = None
    rpi_i2c_add = 17
    AltScan8Callback = None

    CMD_GET_CNT_STATUS = 2

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, master_win, callback):
        print("Entered HwPanel.__init__")
        if not hasattr(self, 'initialized'):
            self.main_win = master_win
            self.AltScan8Callback = callback
            # Use self.AltScan8Callback to call functions on ALT-Scann8 UI
            print(f"hwpanel initialized: win={master_win}, self.main_win={self.main_win}, callback={callback}")
            # Uncomment next line if panel available
            # self._register_to_altscann8()
            self.initialized = True

    def init_completed(self):
        pass
        # Replace pass statement with whatever you want to do at init time

    def shutdown_started(self):
        global ExitingApp
        self.ExitingApp = True
        # Add more statements with whatever you want to do at termination time

    def captured_frame(self):
        pass
        # Replace pass statement with whatever you want to do when a frame is captured

    def film_in_filmgate_warning(self):
        # Mariano: Implement here whichever code is required to determine next step using panel inputs
        # Return 'False' to cancel Rewind/FF operation, 'True' to proceed with it despite the issue
        return False    # By default return cancel

    def film_back_warning(self):
        # Mariano: Implement here whichever code is required to proceed usign panel inputs
        # Return 'False' to cancel film move back operation, 'True' to proceed with it
        return False    # By default return cancel


    """
	****************************************************************************************************************
    Internal functions: Funciona below this point are to be used internally to hw panel module
    Most of them invoke functionallity in ALT-Scann8
	****************************************************************************************************************
    """
	
	
    def _register_to_altscann8(self):
        self.AltScan8Callback(self.HWPANEL_REGISTER, True)

    def _start_stop_scan(self):
        self.AltScan8Callback(self.HWPANEL_START_STOP, None)

    def _fast_forward(self):
        self.AltScan8Callback(self.HWPANEL_FF, None)

    def _rewind(self):
        self.AltScan8Callback(self.HWPANEL_RW, None)

    def _film_forward(self):
        self.AltScan8Callback(self.HWPANEL_FORWARD, None)

    def _film_backward(self):
        self.AltScan8Callback(self.HWPANEL_BACKWARD, None)

    def _focus_view(self):
        self.AltScan8Callback(self.HWPANEL_FOCUS_VIEW, None)

    def _zoom_view(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW, None)

    def _zoom_view_plus(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_PLUS, None)

    def _zoom_view_minus(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_MINUS, None)

    def _zoom_view_right(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_RIGHT, None)

    def _zoom_view_left(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_LEFT, None)

    def _zoom_view_up(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_UP, None)

    def _zoom_view_down(self):
        self.AltScan8Callback(self.HWPANEL_ZOOM_VIEW_DOWN, None)

    def _auto_exposure(self):
        self.AltScan8Callback(self.HWPANEL_AUTO_EXPOSURE, None)

    def _auto_white(self):
        self.AltScan8Callback(self.HWPANEL_AUTO_WB, None)

    def _autoStop_enable(self):
        self.AltScan8Callback(self.HWPANEL_AUTOSTOP_ENABLE, None)

    def _autostop_time(self):
        autostop_time = self.AltScan8Callback(self.HWPANEL_GET_AUTOSTOP_TIME, None)
        # to do something with this value (display on 7 segment led display through HT16K33 on the panel)

    def _set_frame_counter(self):
        self.AltScan8Callback(self.HWPANEL_SET_AUTOSTOP_FRAMES, frame_count)
		# to do something with this value (display on 7 segment led display through HT16K33 on the panel)

    def _set_filmtime(self):
        tilm_time = self.AltScan8Callback(self.HWPANEL_GET_FILM_TIME, None)
        # to do something with this value (display on 7 segment led display through HT16K33 on the panel)

    def _set_fps(self):
        fps = self.AltScan8Callback(self.HWPANEL_GET_FPS, None)
        # to do something with this value (display on 7 segment led display through HT16K33 on the panel)

    def _set_film_S8(self):
        self.AltScan8Callback(self.HWPANEL_SET_FILM_S8, None)

    def _set_film_R8(self):
        self.AltScan8Callback(self.HWPANEL_SET_FILM_R8, None)

    def _set_exposure(self, exp_value):
        self.AltScan8Callback(self.HWPANEL_SET_EXPOSURE, exp_value)

    def _set_red(self, red_value):
        self.AltScan8Callback(self.HWPANEL_SET_WB_RED, red_value)

    def _set_blue(self, blue_value):
        self.AltScan8Callback(self.HWPANEL_SET_WB_BLUE, blue_value)

