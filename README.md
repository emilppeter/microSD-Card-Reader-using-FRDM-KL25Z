# µSD-Card-Reader-using-FRDM-KL25Z
The code to read and write data to µSD card is optimized to reduce the CPU idle time using two approaches.First approach employs the use of FSM wherein the code is broken into many states,with each state meeting its timing budget.Second approach uses CMSIS-RTOS v2 RTX5 to reduce idle time.  
