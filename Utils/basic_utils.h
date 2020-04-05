#ifndef _BASIC_UTILS_H_
#define _BASIC_UTILS_H_

/*
	@DESCRIPTION
	
	TODO
*/




////////////////////////////////////////////////
/////          ASSERT & EXCEPTIONS         /////
////////////////////////////////////////////////

/*=======================================================
* @brief           Asserts that the given condition is held and throws an exception otherwise
*
* @description     This macro is a wrapper for the internal _ASSERT function. 
*                   
*                  The use of a macro is in order to provide _ASSERT with the actual
*                  line,file and function where the assert occoured.
*
* @param           condition  -  The condition to be asserted. 
*
* @param           exception  -  The exeption to be thrown in case the condition evaluates to FALSE.
*
* @return          None
*
* @author          Daniel Greenberger
=========================================================*/
#define ASSERT(condition, exception) do{ _ASSERT((cond), (exception), __FILE__, __LINE__, __FUNC__, #cond) }while(0);


/*=======================================================
* @brief           Internal assert handler.
*
* @description     This function asserts that the given condition is TRUE. 
*                  If the condition evaluates to FALSE:
*                  1. The given exception is thrown.
*                  2. An error message is printed to the stardard output with detailed information about the assert. 
*
* @param           condition     -  A boolean condition to be checked.
*                                
* @param           e             -  An execption to be thrown in case the condition evaluates as FALSE.
*                                
* @param           assert_file   -  Path of the file where the condition was asserted.
*                                
* @param           assert_line   -  The line in assert_file file where the condition was asserted.
*                                
* @param           assert_func   -  The function is which the condition was asserted.
*                                
* @param           assert_text   -  A string describing the condition.
*
* @return          None
*
* @author          Daniel Greenberger
=========================================================*/

inline void __ASSERT (const bool condition, std::exception& e, char* assert_file, 
                            uint32_t assert_line, const char* assert_func, const char* assert_text )
{
    if unlikely(!cond)
    {
        std::cerr << "Assrtation failed at " << assert_func << "(" << assert_file << ":" << assert_line <<"):   " << assert_text;
        throw e;
    }
}

////////////////////////////////////////////////
/////       BIT MANIPULATION & FLAGS       /////
////////////////////////////////////////////////


/*=======================================================
* @brief           Generates bitmask for the given bit. 
*
* @param           i  -  Bit number to create the mask for. 
*                        For example:  i=3  ->  output will be the binary value 0b00000000000000000000000000001000
*
* @return          Bitmask of the provided bit.
*
* @author          Daniel Greenberger
=========================================================*/
constexpr uint_32 __BIT(uint_32 i)
{
    static_assert(i <= 31, "__BIT:  Bit value must be in the range:     0 <= bit_value <= 31");
    return (1UL << i);
}




////////////////////////////////////////////////
/////         CONCURRENCY & THREADS        /////
////////////////////////////////////////////////


/*=======================================================
* @brief           Macro for critical-section code
*
* @description     This macro is a useful for invoking code which needs to be performed in critical section. 
*                  In order to use this macro, you must define a wrapper for a specific mutex as shown in
*                  the example below (MY_MODULE_CRITICAL_SECTION)
*
* @param           mutex  -  The mutex to be used to protect the critical section.
*                            The mutex will be acquired (before) and released  (after) performing  
*                            the critical section code. 
* 
*                  code   -  Code to be performed in critical section, which can be any valid syntax code.
*
*                            NOTE:
*                            The code will be performed in it's own scope, so any variable defined inside 
*                            a critical section will go out-of-scope after the mutex is released. 
* 
* @return          None
*
* @author          Daniel Greenberger
=========================================================*/
#define __CRITICAL_SECTION(mutex, code)     do{   mutex.lock();  code;   m_mutex.unlock();    } while(0);

/*
    This is an example wrapper for the critical section macro. 
    DO NOT USE THIS MACRO DIRECTLY. 
    Instead, define your own wrapper with a meaningful name and exclusive mutex.
*/
#define MY_MODULE_CRITICAL_SECTION(code)   do{  __CRITICAL_SECTION(my_mutex, code)  } while(0); 







#endif /*_BASIC_UTILS_H_*/
