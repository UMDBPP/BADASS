function stop_serial_monitor()


    stop(timerfind);
    delete(timerfind)
    evalin('base','clear timer_obj');

end