
#---Row 111
hw_panel_installed = True
hwpanel_registered = False
if hw_panel_installed:
    try:
        from hw_panel import HwPanel
        hw_panel_installed = True
        import RPi.GPIO as GPIO
    except Exception as e:
        print(f"Hw panel import issue: {e}")
        hw_panel_installed = False



#---Row 314
# Panel 'buttons'
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



#---Row 627
def exit_app(do_save):  # Exit Application
    global win
    global ExitingApp
    global hw_panel

    log_current_session()   # Before exiting, write session data to disk

    # *** ALT-Scann8 shutdown starts ***
    if hwpanel_registered:
        hw_panel.shutdown_started()

    win.config(cursor="watch")
    win.update()
    # Flag app is exiting for all outstanding afters to expire
    ExitingApp = True
    if onesec_after != 0:
        win.after_cancel(onesec_after)
    if arduino_after != 0:
        win.after_cancel(arduino_after)
    # Terminate threads
    if not SimulatedRun and not CameraDisabled:
        capture_display_event.set()
        capture_save_event.set()
        capture_display_queue.put(END_TOKEN)
        capture_save_queue.put(END_TOKEN)
        capture_save_queue.put(END_TOKEN)
        capture_save_queue.put(END_TOKEN)

        while active_threads > 0:
            win.update()
            logging.debug(f"Waiting for threads to exit, {active_threads} pending")
            time.sleep(0.2)

    # Uncomment next two lines when running on RPi
    if not SimulatedRun:
        send_arduino_command(CMD_TERMINATE)  # Tell Arduino we stop (to turn off uv led
        # Close preview if required
        if not CameraDisabled:
            if PiCam2PreviewEnabled:
                camera.stop_preview()
            camera.close()
    # Set window position for next run
    ConfigData["WindowPos"] = win.geometry()
    ConfigData["AutoStopActive"] = AutoStopEnabled
    ConfigData["AutoStopType"] = autostop_type.get()
    if frames_to_go_str.get() == '':
        ConfigData["FramesToGo"] = -1
    # Write session data upon exit
    if do_save:
        save_configuration_data_to_disk()

    win.config(cursor="")

    win.destroy()



#---Row 1621
def cmd_retreat_movie():
    global RetreatMovieActive

    if not RetreatMovieActive:
        if hwpanel_registered:
            popup_window = custom_messagebox_yes_no(title='Move film back',
                                            message="This operation requires manually moving the source reel to collect film being moved back. "
                                            "If you don't, film will probably end up jammed in the film gate. "
                                            "Are you sure you want to proceed?"
                                            "\r\n(Please accept using panel physical buttons)")
            win.update_idletasks()
            confirm = hw_panel.film_back_warning()
            if SimulatedRun:
                time.sleep(5)
            popup_window.destroy()
        else:
            confirm = tk.messagebox.askyesno(title="Move film back",
                                            message="This operation requires manually moving the source reel to collect film being moved back. "
                                            "If you don't, film will probably end up jammed in the film gate."
                                            "\r\nAre you sure you want to proceed?")
        if not confirm:
            return

    # Update button text
    if not RetreatMovieActive:  # Advance movie is about to start...
        retreat_movie_btn.config(text='■', bg='red', fg='white', 
                                 relief=SUNKEN)  # ...so now we propose to stop it in the button test
    else:
        retreat_movie_btn.config(text='◀', bg=save_bg, fg=save_fg,
                                 relief=RAISED)  # Otherwise change to default text to start the action
    RetreatMovieActive = not RetreatMovieActive
    # Send instruction to Arduino
    if not SimulatedRun:
        send_arduino_command(CMD_FILM_BACKWARD)

    # Enable/Disable related buttons
    except_widget_global_enable([retreat_movie_btn], not RetreatMovieActive)



#---Row 1660
def cmd_rewind_movie():
    global win
    global RewindMovieActive
    global RewindErrorOutstanding, RewindEndOutstanding

    if SimulatedRun and RewindMovieActive:  # no callback from Arduino in simulated mode
        RewindEndOutstanding = True

    # Before proceeding, get confirmation from user that fild is correctly routed
    if not RewindMovieActive:  # Ask only when rewind is not ongoing
        RewindMovieActive = True
        # Update button text
        rewind_btn.config(text='■', bg='red', fg='white', font=("Arial", FontSize + 3), 
                          relief=SUNKEN)  # ...so now we propose to stop it in the button test
        # Enable/Disable related buttons
        except_widget_global_enable([rewind_btn], not RewindMovieActive)
        # Invoke rewind_loop to continue processing until error or end event
        win.after(5, rewind_loop)
    elif RewindErrorOutstanding:
        if hwpanel_registered:
            popup_window = custom_messagebox_yes_no(title='Error during rewind',
                                     message='It seems there is film loaded via filmgate. Are you sure you want to proceed?\
                                     \r\n(Please accept or cancel using panel physical buttons)')
            win.update_idletasks()
            confirm = hw_panel.film_in_filmgate_warning()
            popup_window.destroy()
        else:
            confirm = tk.messagebox.askyesno(title='Error during rewind',
                                            message='It seems there is film loaded via filmgate. \
                                            \r\nAre you sure you want to proceed?')
        if confirm:
            time.sleep(0.2)
            if not SimulatedRun:
                send_arduino_command(CMD_UNCONDITIONAL_REWIND)  # Forced rewind, no filmgate check
                # Invoke fast_forward_loop a first time when fast-forward starts
                win.after(5, rewind_loop)
        else:
            RewindMovieActive = False
    elif RewindEndOutstanding:
        RewindMovieActive = False

    if not RewindMovieActive:
        rewind_btn.config(text='◀◀', bg=save_bg, fg=save_fg, font=("Arial", FontSize + 3), 
                          relief=RAISED)  # Otherwise change to default text to start the action
        # Enable/Disable related buttons
        except_widget_global_enable([rewind_btn], not RewindMovieActive)

    if not RewindErrorOutstanding and not RewindEndOutstanding:  # invoked from button
        time.sleep(0.2)
        if not SimulatedRun:
            send_arduino_command(CMD_REWIND)

    if RewindErrorOutstanding:
        RewindErrorOutstanding = False
    if RewindEndOutstanding:
        RewindEndOutstanding = False



#---Row 1728
def cmd_fast_forward_movie():
    global win
    global FastForwardActive
    global FastForwardErrorOutstanding, FastForwardEndOutstanding

    if SimulatedRun and FastForwardActive:  # no callback from Arduino in simulated mode
        FastForwardEndOutstanding = True

    # Before proceeding, get confirmation from user that fild is correctly routed
    if not FastForwardActive:  # Ask only when rewind is not ongoing
        FastForwardActive = True
        # Update button text
        fast_forward_btn.config(text='■', bg='red', fg='white', font=("Arial", FontSize + 3), relief=SUNKEN)
        # Enable/Disable related buttons
        except_widget_global_enable([fast_forward_btn], not FastForwardActive)
        # Invoke fast_forward_loop a first time when fast-forward starts
        win.after(5, fast_forward_loop)
    elif FastForwardErrorOutstanding:
        if hwpanel_registered:
            popup_window = custom_messagebox_yes_no(title='Error during fast forward',
                                     message='It seems there is film loaded via filmgate. Are you sure you want to proceed?\
                                     \r\n(Please accept or cancel using panel physical buttons)')
            win.update_idletasks()
            confirm = hw_panel.film_in_filmgate_warning()
            popup_window.destroy()
        else:
            confirm = tk.messagebox.askyesno(title='Error during fast forward',
                                            message='It seems there is film loaded via filmgate. \
                                            \r\nAre you sure you want to proceed?')
        if confirm:
            time.sleep(0.2)
            if not SimulatedRun:
                send_arduino_command(CMD_UNCONDITIONAL_FAST_FORWARD)  # Forced FF, no filmgate check
                # Invoke fast_forward_loop a first time when fast-forward starts
                win.after(5, fast_forward_loop)
        else:
            FastForwardActive = False
    elif FastForwardEndOutstanding:
        FastForwardActive = False

    if not FastForwardActive:
        fast_forward_btn.config(text='▶▶', bg=save_bg, fg=save_fg, font=("Arial", FontSize + 3), relief=RAISED)
        # Enable/Disable related buttons
        except_widget_global_enable([fast_forward_btn], not FastForwardActive)

    if not FastForwardErrorOutstanding and not FastForwardEndOutstanding:  # invoked from button
        time.sleep(0.2)
        if not SimulatedRun:
            send_arduino_command(CMD_FAST_FORWARD)

    if FastForwardErrorOutstanding:
        FastForwardErrorOutstanding = False
    if FastForwardEndOutstanding:
        FastForwardEndOutstanding = False



#---Row 2521
def capture_single(mode):
    global CurrentFrame
    global total_wait_time_save_image, PreviewModuleValue
    global hw_panel

    # *** ALT-Scann8 capture frame ***
    if hwpanel_registered:
        hw_panel.captured_frame()

    is_dng = FileType == 'dng'
    is_png = FileType == 'png'
    curtime = time.time()
    if not DisableThreads:
        if is_dng or is_png:  # Save as request only for DNG captures
            request = camera.capture_request(capture_config)
            # For PiCamera2, preview and save to file are handled in asynchronous threads
            if CurrentFrame % PreviewModuleValue == 0:
                captured_image = request.make_image('main')
                # Display preview using thread, not directly
                queue_item = tuple((IMAGE_TOKEN, captured_image, CurrentFrame, 0))
                capture_display_queue.put(queue_item)
            else:
                time_preview_display.add_value(0)
            if mode == 'normal' or mode == 'manual':  # Do not save in preview mode, only display
                save_queue_item = tuple((REQUEST_TOKEN, request, CurrentFrame, 0))
                capture_save_queue.put(save_queue_item)
                logging.debug(f"Queueing frame ({CurrentFrame}")
        else:
            captured_image = camera.capture_image("main")
            if NegativeImage:
                captured_image = reverse_image(captured_image)
            queue_item = tuple((IMAGE_TOKEN, captured_image, CurrentFrame, 0))
            # For PiCamera2, preview and save to file are handled in asynchronous threads
            if CurrentFrame % PreviewModuleValue == 0:
                # Display preview using thread, not directly
                capture_display_queue.put(queue_item)
            else:
                time_preview_display.add_value(0)
            if mode == 'normal' or mode == 'manual':  # Do not save in preview mode, only display
                capture_save_queue.put(queue_item)
                logging.debug(f"Queuing frame {CurrentFrame}")
        if mode == 'manual':  # In manual mode, increase CurrentFrame
            CurrentFrame += 1
            # Update number of captured frames
            Scanned_Images_number.set(CurrentFrame)
    else:
        if is_dng or is_png:
            request = camera.capture_request(capture_config)
            if CurrentFrame % PreviewModuleValue == 0:
                captured_image = request.make_image('main')
            else:
                captured_image = None
            draw_preview_image(captured_image, CurrentFrame, 0)
            if mode == 'normal' or mode == 'manual':  # Do not save in preview mode, only display
                request.save_dng(FrameFilenamePattern % (CurrentFrame, FileType))
                logging.debug(f"Saving DNG frame ({CurrentFrame}: {round((time.time() - curtime) * 1000, 1)}")
            request.release()
        else:
            captured_image = camera.capture_image("main")
            if NegativeImage:
                captured_image = reverse_image(captured_image)
            draw_preview_image(captured_image, CurrentFrame, 0)
            captured_image.save(FrameFilenamePattern % (CurrentFrame, FileType), quality=95)
            logging.debug(
                f"Saving image ({CurrentFrame}: {round((time.time() - curtime) * 1000, 1)}")
        aux = time.time() - curtime
        total_wait_time_save_image += aux
        time_save_image.add_value(aux)
        if mode == 'manual':  # In manual mode, increase CurrentFrame
            CurrentFrame += 1
            # Update number of captured frames
            Scanned_Images_number.set(CurrentFrame)



#---Row 4429
# HwPanel callback function
# Used to invoke ALT-Scann8 functions from HwPanel extension
def hw_panel_callback(command, param1=None):
    global hwpanel_registered
    if command == HWPANEL_REGISTER:
        hwpanel_registered = param1
        logo_label.config(bg="light blue")
    elif command == HWPANEL_START_STOP:
        start_scan()
    elif command == HWPANEL_FORWARD:
        cmd_advance_movie()
    elif command == HWPANEL_BACKWARD:
        cmd_retreat_movie()
    elif command == HWPANEL_FF:
        cmd_fast_forward_movie()
    elif command == HWPANEL_RW:
        cmd_rewind_movie()
    elif command == HWPANEL_FOCUS_VIEW:
        real_time_display.set(not RealTimeDisplay)
        cmd_set_real_time_display()
    elif command == HWPANEL_ZOOM_VIEW:
        real_time_zoom.set(not RealTimeZoom)
        cmd_set_focus_zoom()
    elif command == HWPANEL_ZOOM_VIEW_PLUS:
        cmd_set_focus_plus()
    elif command == HWPANEL_ZOOM_VIEW_MINUS:
        cmd_set_focus_minus()
    elif command == HWPANEL_ZOOM_VIEW_RIGHT:
        cmd_set_focus_right()
    elif command == HWPANEL_ZOOM_VIEW_LEFT:
        cmd_set_focus_left()
    elif command == HWPANEL_ZOOM_VIEW_UP:
        cmd_set_focus_up()
    elif command == HWPANEL_ZOOM_VIEW_DOWN:
        cmd_set_focus_down()
    elif command == HWPANEL_AUTO_EXPOSURE:
        AE_enabled.set(not AutoExpEnabled)
        cmd_set_auto_exposure()
    elif command == HWPANEL_AUTO_WB:
        AWB_enabled.set(not AutoWbEnabled)
        cmd_set_auto_wb()
    elif command == HWPANEL_AUTOSTOP_ENABLE:
        auto_stop_enabled.set(not AutoStopEnabled)
        cmd_set_auto_stop_enabled()
    elif command == HWPANEL_GET_AUTOSTOP_TIME:
        return frames_to_go_time_str.get()
    elif command == HWPANEL_SET_AUTOSTOP_FRAMES:
        frames_to_go_str.set(param1)
    elif command == HWPANEL_GET_FILM_TIME:
        return scanned_Images_time_value.get()
    elif command == HWPANEL_GET_FPS:
        return scanned_Images_fps_value.get()
    elif command == HWPANEL_SET_FILM_S8:
        cmd_set_s8()
    elif command == HWPANEL_SET_FILM_R8:
        cmd_set_r8()
    elif command == HWPANEL_SET_EXPOSURE:
        exposure_value.set(param1)
        cmd_exposure_selection()
    elif command == HWPANEL_SET_WB_RED:
        wb_red_value.set(param1)
        cmd_wb_red_selection()
    elif command == HWPANEL_SET_WB_BLUE:
        wb_blue_value.set(param1)
        cmd_wb_blue_selection()



#---Row 4495
def tscann8_init():
    global win
    global camera
    global i2c
    global CurrentDir
    global ZoomSize
    global capture_display_queue, capture_display_event
    global capture_save_queue, capture_save_event
    global MergeMertens, camera_resolutions
    global active_threads
    global time_save_image, time_preview_display, time_awb, time_autoexp, offset_image
    global hw_panel

    if SimulatedRun:
        logging.info("Not running on Raspberry Pi, simulated run for UI debugging purposes only")
    else:
        logging.info("Running on Raspberry Pi")

    if not can_check_dng_frames_for_misalignment:
        logging.warning("Frame alignment for DNG files is disabled. To enable it please install rawpy library")

    logging.debug("BaseFolder=%s", BaseFolder)

    if not SimulatedRun:
        i2c = smbus.SMBus(1)
        # Set the I2C clock frequency to 400 kHz
        i2c.write_byte_data(16, 0x0F, 0x46)  # I2C_SCLL register
        i2c.write_byte_data(16, 0x10, 0x47)  # I2C_SCLH register

    if not SimulatedRun and not CameraDisabled:  # Init PiCamera2 here, need resolution list for drop down
        camera = Picamera2()
        camera_resolutions = CameraResolutions(camera.sensor_modes)
        logging.info(f"Camera Sensor modes: {camera.sensor_modes}")
        PiCam2_configure()
        ZoomSize = camera.capture_metadata()['ScalerCrop']
        logging.debug(f"ScalerCrop: {ZoomSize}")
    if SimulatedRun:
        # Initializes resolution list from a hardcoded sensor_modes
        camera_resolutions = CameraResolutions(simulated_sensor_modes)

    # Initialize rolling average objects
    time_save_image = RollingAverage(50)
    time_preview_display = RollingAverage(50)
    time_awb = RollingAverage(50)
    time_autoexp = RollingAverage(50)
    offset_image = RollingAverage(5)

    create_main_window()

    # Check if hw panel module available
    if hw_panel_installed:
        hw_panel = HwPanel(win, hw_panel_callback)
    else:
        hw_panel = None

    # Init default steps per frame (used by manual scan and VFD)
    adjust_default_frame_steps()

    # Init HDR variables
    hdr_init()
    # Create MergeMertens Object for HDR
    MergeMertens = cv2.createMergeMertens()

    reset_controller()

    get_controller_version()

    send_arduino_command(CMD_REPORT_PLOTTER_INFO, PlotterEnabled)

    win.update_idletasks()

    if not SimulatedRun and not CameraDisabled:
        # JRE 20/09/2022: Attempt to speed up overall process in PiCamera2 by having captured images
        # displayed in the preview area by a dedicated thread, so that time consumed in this task
        # does not impact the scan process speed
        capture_display_queue = queue.Queue(maxsize=MaxQueueSize)
        capture_display_event = threading.Event()
        capture_save_queue = queue.Queue(maxsize=MaxQueueSize)
        capture_save_event = threading.Event()
        display_thread = threading.Thread(target=capture_display_thread, args=(capture_display_queue,
                                                                               capture_display_event, 0))
        save_thread_1 = threading.Thread(target=capture_save_thread, args=(capture_save_queue, capture_save_event, 1))
        save_thread_2 = threading.Thread(target=capture_save_thread, args=(capture_save_queue, capture_save_event, 2))
        save_thread_3 = threading.Thread(target=capture_save_thread, args=(capture_save_queue, capture_save_event, 3))
        active_threads += 4
        display_thread.start()
        save_thread_1.start()
        save_thread_2.start()
        save_thread_3.start()
        logging.debug("Threads initialized")

    logging.debug("ALT-Scann 8 initialized")




#---Row 6694
def main(argv):
    global SimulatedRun, SimulatedArduinoVersion
    global ExpertMode, ExperimentalMode, PlotterEnabled
    global LogLevel, LoggingMode
    global ALT_scann_init_done
    global CameraDisabled, DisableThreads
    global FontSize, UIScrollbars
    global WidgetsEnabledWhileScanning
    global DisableToolTips
    global win, hw_panel
    global UserConsent, ConfigData, LastConsentDate

    DisableToolTips = False
    goanyway = False

    try:
        opts, args = getopt.getopt(argv, "sexl:phntwf:ba:", ["goanyway"])
    except getopt.GetoptError as e:
        print(f"Invalid command line parameter: {e}")
        return

    for opt, arg in opts:
        if opt == '-s':
            SimulatedRun = True
        elif opt == '-e':
            ExpertMode = not ExpertMode
        elif opt == '-x':
            ExperimentalMode = not ExperimentalMode
        elif opt == '-d':
            CameraDisabled = True
        elif opt == '-l':
            LoggingMode = arg
        elif  opt == '-a':
            SimulatedArduinoVersion = arg
        elif opt == '-f':
            FontSize = int(arg)
        elif opt == '--goanyway':
            goanyway = True
        elif opt == '-b':
            UIScrollbars = True
        elif opt == '-n':
            DisableToolTips = True
        elif opt == '-t':
            DisableThreads = True
        elif opt == '-w':
            WidgetsEnabledWhileScanning = not WidgetsEnabledWhileScanning
        elif opt == '-h':
            print("ALT-Scann 8 Command line parameters")
            print("  -s             Start Simulated session")
            print("  -e             Activate expert mode")
            print("  -x             Activate experimental mode")
            print("  -d             Disable camera (for development purposes)")
            print("  -n             Disable Tooltips")
            print("  -t             Disable multi-threading")
            print("  -f <size>      Set user interface font size (11 by default)")
            print("  -b             Add scrollbars to UI (in case it does not fit)")
            print("  -w             Keep control widgets enabled while scanning")
            print("  -l <log mode>  Set log level (standard Python values (DEBUG, INFO, WARNING, ERROR)")
            exit()

    if goanyway:
        print("Work in progress, version not usable yet.")
        tk.messagebox.showerror("WIP", "Work in progress, version not usable yet.")
        return
    # Set our CWD to the same folder where the script is. 
    # Otherwise webbrowser failt to launch (cannot open path of the current working directory: Permission denied)
    os.chdir(ScriptDir) 

    LogLevel = getattr(logging, LoggingMode.upper(), None)
    if not isinstance(LogLevel, int):
        raise ValueError('Invalid log level: %s' % LogLevel)
    else:
        init_logging()

    ALT_scann_init_done = False

    if not numpy_loaded:
        logging.error("Numpy library could no tbe loaded.\r\nPlease install it with this command 'sudo apt install python3-numpy'.")
        return

    win = tkinter.Tk()  # Create temporary main window to support popups before main window is created
    win.withdraw()  # Hide temporary main window

    load_configuration_data_from_disk()  # Read json file in memory, to be processed by 'load_session_data_post_init'

    if not validate_config_folders():
        return

    load_config_data_pre_init()

    init_user_count_data()

    # Check reporting consent on first run
    get_consent()

    tscann8_init()

    if not SimulatedRun:
        arduino_listen_loop()

    if DisableToolTips:
        as_tooltips.disable()

    load_session_data_post_init()

    init_multidependent_widgets()

    if SimulatedRun:
        logging.debug("Starting in simulated mode.")
    if ExpertMode:
        logging.debug("Toggle expert mode.")
    if ExperimentalMode:
        logging.debug("Toggle experimental mode.")
    if CameraDisabled:
        logging.debug("Camera disabled.")
    if FontSize != 0:
        logging.debug(f"Font size = {FontSize}")
    if DisableThreads:
        logging.debug("Threads disabled.")

    ALT_scann_init_done = True

    refresh_qr_code()

    # Write environment info to log
    data = generate_qr_code_info()
    logging.info(data)

    report_usage()

    # *** ALT-Scann8 load complete ***

    if hw_panel_installed:
        hw_panel.init_completed()

    onesec_periodic_checks()

    # Main Loop
    win.mainloop()  # running the loop that works as a trigger

    if not SimulatedRun and not CameraDisabled:
        camera.close()


if __name__ == '__main__':
    main(sys.argv[1:])

#--- END of  ALT-Scann code ---




	
	