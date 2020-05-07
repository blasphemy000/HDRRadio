SimpleButton
=============

This code was originally based on the button code inside of
the 'ClickEncoder' class written by 0xPIT (karl@pitrich.com).
At this point the code has be rewritten and modified to the
point where I would consider it to only be inspired by the
original source material. I am including this out of respect
for the original source author so that anyone that would like
to seek out the inspiration for this code is able to do so.

While the isr_DoPolling routine can be invoked using your choice
of timer code, the timer system that I use is 'TimerOne' with
links provided below. No code from that library is included
in my 'SimpleButton' library as the polling timer is up to the
user and/or developer to decide upon for themselves.

TimerOne: http://playground.arduino.cc/Code/Timer1
          : https://github.com/PaulStoffregen/TimerOne

ClickEncoder: https://github.com/0xPIT/encoder
