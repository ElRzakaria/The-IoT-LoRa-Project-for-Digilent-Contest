/**
 * UART library
 * author: Guillaume Patrigeon -- autogenerated
 * update: 10-12-2018
 */

#ifndef __UART_H__
#define	__UART_H__


//--------------------------------------------------------
#if defined(UART1) && defined(_USE_UART1)

/// Initialize UART1 module
void UART1_Init(void);


/// Enable UART1 module
#define UART1_Enable()          UART1.PE = 1


/// Disable UART1 module
#define UART1_Disable()         UART1.PE = 0


/// Set UART1 module baudrate
void UART1_SetBaudrate(unsigned int baudrate);


/// Load a byte into TX buffer
void UART1_Send(const char c);


/// Check if there is data in RX buffer, return 1 if there is at last 1 byte in buffer
int UART1_IsRxNotEmpty(void);


/// Read the first byte in the RX buffer
char UART1_Read(void);


/// Empty both TX and RX buffers
void UART1_Clean(void);


/// Get the number of bytes in TX buffer
int UART1_GetTxCount(void);


/// Get the number of bytes in RX buffer
int UART1_GetRxCount(void);

#endif



//--------------------------------------------------------
#if defined(UART2) && defined(_USE_UART2)

/// Initialize UART2 module
void UART2_Init(void);


/// Enable UART2 module
#define UART2_Enable()          UART2.PE = 1


/// Disable UART2 module
#define UART2_Disable()         UART2.PE = 0


/// Set UART2 module baudrate
void UART2_SetBaudrate(unsigned int baudrate);


/// Load a byte into TX buffer
void UART2_Send(const char c);


/// Check if there is data in RX buffer, return 1 if there is at last 1 byte in buffer
int UART2_IsRxNotEmpty(void);


/// Read the first byte in the RX buffer
char UART2_Read(void);


/// Empty both TX and RX buffers
void UART2_Clean(void);


/// Get the number of bytes in TX buffer
int UART2_GetTxCount(void);


/// Get the number of bytes in RX buffer
int UART2_GetRxCount(void);

#endif



//--------------------------------------------------------
#if defined(UART3) && defined(_USE_UART3)

/// Initialize UART3 module
void UART3_Init(void);


/// Enable UART3 module
#define UART3_Enable()          UART3.PE = 1


/// Disable UART3 module
#define UART3_Disable()         UART3.PE = 0


/// Set UART3 module baudrate
void UART3_SetBaudrate(unsigned int baudrate);


/// Load a byte into TX buffer
void UART3_Send(const char c);


/// Check if there is data in RX buffer, return 1 if there is at last 1 byte in buffer
int UART3_IsRxNotEmpty(void);


/// Read the first byte in the RX buffer
char UART3_Read(void);


/// Empty both TX and RX buffers
void UART3_Clean(void);


/// Get the number of bytes in TX buffer
int UART3_GetTxCount(void);


/// Get the number of bytes in RX buffer
int UART3_GetRxCount(void);

#endif



//--------------------------------------------------------
#if defined(UART4) && defined(_USE_UART4)

/// Initialize UART4 module
void UART4_Init(void);


/// Enable UART4 module
#define UART4_Enable()          UART4.PE = 1


/// Disable UART4 module
#define UART4_Disable()         UART4.PE = 0


/// Set UART4 module baudrate
void UART4_SetBaudrate(unsigned int baudrate);


/// Load a byte into TX buffer
void UART4_Send(const char c);


/// Check if there is data in RX buffer, return 1 if there is at last 1 byte in buffer
int UART4_IsRxNotEmpty(void);


/// Read the first byte in the RX buffer
char UART4_Read(void);


/// Empty both TX and RX buffers
void UART4_Clean(void);


/// Get the number of bytes in TX buffer
int UART4_GetTxCount(void);


/// Get the number of bytes in RX buffer
int UART4_GetRxCount(void);

#endif



#endif
