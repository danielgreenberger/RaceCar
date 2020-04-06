#ifndef _BASIC_UTILS_H_
#define _BASIC_UTILS_H_

/*
	@DESCRIPTION
	
	This file includes some basic utilities which can be usefull across the entire project. 
*/



///////////////////////////////////////////////////
/////         PROCESSOR && OPTIMIZATION       /////
///////////////////////////////////////////////////


/*=======================================================
* @brief           Optimization macros for better branch predition
*
* @description     These macros provide a hint for the compiler on the expected value of x.
*                  This does not impact the correctness of the expression evaluation, 
*                  however it improves perfomance in cases where the expression was evaluated as 
*                  expected (true for _Usually, false for _Rarely).
*
*                  These macros can be used to optimize performence when the value of a boolean
*                  condition is almost always the same value. 
*
* @param           x  -  A boolean condition (or an integer expression to be evaluated as boolean)
*
* @return          The same value as x. 
*
* @author          Daniel Greenberger
=========================================================*/
#define _Usually(x)      __builtin_expect(!!(x), 1) 
#define _Rarely(x)    __builtin_expect(!!(x), 0) 




////////////////////////////////////////////////
/////            STANDARD OUTPUT           /////
////////////////////////////////////////////////
namespace TextFormatting
{

/*
    Text formatting options for output terminals supporting the ANSI standard (i.e Bash). 
    
    For more info visit:
    https://en.wikipedia.org/wiki/ANSI_escape_code#Colors
    http://www.termsys.demon.co.uk/vtansi.htm#colors

*/


enum DisplayAttributes
{
    /* Misc */
    Reset_all_fornatting  = 0,
    Bright                = 1,
    Dim                   = 2,
    Underscore            = 4,
    Blink                 = 5,
    Reverse               = 7,
    Hidden                = 8,

    /*	Foreground (text) Colors */
    Foreground_Black      = 30,
    Foreground_Red        = 31,
    Foreground_Green      = 32,
    Foreground_Yellow     = 33,
    Foreground_Blue       = 34,
    Foreground_Magenta    = 35,
    Foreground_Cyan       = 36,
    Foreground_White      = 37,

    /*	Background Colors */
    Background_Black      = 40,
    Background_Red        = 41,
    Background_Green      = 42,
    Background_Yellow     = 43,
    Background_Blue       = 44,
    Background_Magenta    = 45,
    Background_Cyan       = 46,
    Background_White      = 47,
};


/*
    Meta strings for bash formats parsing
*/
const std::string FORMAT_BEGIN_STR = "\033[";
const std::string FORMAT_END_STR = "m";


/*=======================================================
* @brief           Begin text formatting
=========================================================*/
inline std::string BASH_FORMATTING_BEGIN(int attribute_val)
{
    return FORMAT_BEGIN_STR + std::to_string(attribute_val) + FORMAT_END_STR;
}

/*=======================================================
* @brief           End text formatting
=========================================================*/
inline std::string BASH_FORMATTING_END()
{
    return FORMAT_BEGIN_STR + FORMAT_END_STR;
}

/*=======================================================
* @brief           Format text
=========================================================*/
inline std::string FORMAT(DisplayAttributes att, std::string text)
{    
    return BASH_FORMATTING_BEGIN(att) + text + BASH_FORMATTING_END();
}


/*
    The following functions are example uses of the formatting function
    for common cases. 
*/




/*==========================================================================
* @brief           Format input text to be shown as red in the bash terminal
============================================================================*/
inline std::string MARK_RED(std::string text)
{    
    return FORMAT(DisplayAttributes::Foreground_Red, text);
}

/*==========================================================================
* @brief           Format input text to be shown as bold blue in the bash terminal
============================================================================*/
inline std::string MARK_BOLD_BLUE(std::string text)
{
    std:string output = text;
    
    output = FORMAT(DisplayAttributes::Foreground_Blue, output);
    output = FORMAT(DisplayAttributes::Bright, output);
    
    return output;
}

/*==========================================================================
* @brief           Format input text to be shown with bottom underscore
============================================================================*/
inline std::string MARK_UNDERSCORE(std::string text)
{    
    return FORMAT(DisplayAttributes::Underscore, text);
}


} // namespace TextFormatting
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
#define ASSERT(condition, exception)                                                                                  \
do                                                                                                                    \
{                                                                                                                     \
    const char* condition_string = #condition;                                                                        \
    __ASSERT((condition), (exception), __FILE__, __LINE__, __PRETTY_FUNCTION__, condition_string);                    \
}                                                                                                                     \
while(0);                                                                                                             


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

inline void __ASSERT (const bool condition, const std::exception& e, const char* assert_file, 
                            uint32_t assert_line, const char* assert_func, const char* assert_text )
{
    if (_Rarely(!condition))
    {
        // Print message to standard error output
        std::cerr << std::endl;
        std::cerr << TextFormatting::MARK_RED("Error: ");
        std::cerr << "Assertation failed at "<< std::endl;
        std::cerr << TextFormatting::MARK_BOLD_BLUE("\t file:      ") << assert_file << std::endl;
        std::cerr << TextFormatting::MARK_BOLD_BLUE("\t line:      ") << assert_line << std::endl;
        std::cerr << TextFormatting::MARK_BOLD_BLUE("\t function:  ") << assert_func << std::endl;
        std::cerr << TextFormatting::MARK_BOLD_BLUE("\t condition: ") << assert_text << std::endl;
        

        // Throw the exception
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
*                        Bit value must be in the range:     0 <= bit_value <= 31 to prevent overflow.
*
* @return          Bitmask of the provided bit.
*
* @author          Daniel Greenberger
=========================================================*/
constexpr uint32_t __BIT(int i)
{
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
#define __CRITICAL_SECTION(mutex, code)     do{   mutex.lock();  code;   mutex.unlock();    } while(0);

/*
    This is an example wrapper for the critical section macro. 
    DO NOT USE THIS MACRO DIRECTLY. 
    Instead, define your own wrapper with a meaningful name and exclusive mutex.
*/
#define MY_MODULE_CRITICAL_SECTION(code)   do{  __CRITICAL_SECTION(my_mutex, code)  } while(0); 







#endif /*_BASIC_UTILS_H_*/
