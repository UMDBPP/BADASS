function stop_serial_monitor(t)


    stop(t);
    delete(t);
    evalin('base','clear timer_obj');

end