**-a**, **--auto** *us*

        Set the automatic trace mode. This mode sets some commonly used options
        while debugging the system. It is equivalent to use **-T** *us* **-s** *us*
        **-t**. By default, *timerlat* tracer uses FIFO:95 for *timerlat* threads,
        thus equilavent to **-P** *f:95*.

**-p**, **--period** *us*

        Set the *timerlat* tracer period in microseconds.

**-i**, **--irq** *us*

        Stop trace if the *IRQ* latency is higher than the argument in us.

**-T**, **--thread** *us*

        Stop trace if the *Thread* latency is higher than the argument in us.

**-s**, **--stack** *us*

        Save the stack trace at the *IRQ* if a *Thread* latency is higher than the
        argument in us.

**-t**, **--trace** \[*file*]

        Save the stopped trace to [*file|timerlat_trace.txt*].

**--dma-latency** *us*
        Set the /dev/cpu_dma_latency to *us*, aiming to bound exit from idle latencies.
        *cyclictest* sets this value to *0* by default, use **--dma-latency** *0* to have
        similar results.

**-k**, **--kernel-threads**

        Use timerlat kernel-space threads, in contrast of **-u**.

**-u**, **--user-threads**

        Set timerlat to run without a workload, and then dispatches user-space workloads
        to wait on the timerlat_fd. Once the workload is awakes, it goes to sleep again
        adding so the measurement for the kernel-to-user and user-to-kernel to the tracer
        output. **--user-threads** will be used unless the user specify **-k**.

**-U**, **--user-load**

        Set timerlat to run without workload, waiting for the user to dispatch a per-cpu
        task that waits for a new period on the tracing/osnoise/per_cpu/cpu$ID/timerlat_fd.
        See linux/tools/rtla/sample/timerlat_load.py for an example of user-load code.
