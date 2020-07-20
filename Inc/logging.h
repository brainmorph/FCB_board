/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOGGING_H_
#define LOGGING_H_

#ifdef __cplusplus
 extern "C" {
#endif


// Increase the total system error count
void log_incrementErrorCount();

// Return the total system errors encountered so far
int log_totalErrorCount();


#endif /* LOGGING_H_ */
