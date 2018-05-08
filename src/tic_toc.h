#ifndef TIC_TOC_H
#define TIC_TOC_H


#include <iostream>
#include <chrono>

// pulled from : http://programming-tips-and-tricks.blogspot.com/2017/05/tic-and-toc-functions-in-c-millisecond.html



typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

static Clock::time_point t0 = Clock::now();

void tic()
{
 t0 = Clock::now();
}


void toc()
{
    Clock::time_point t1 = Clock::now();
    milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
    std::cout <<"Elapsed time is "<< ms.count() << " milliseconds\n";
}

#endif