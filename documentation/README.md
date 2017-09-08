# Documentation
We are trying to put a really concerted effort into bettering our documentation process this year. As is such, writing good documentation articles is **Really** important. As a member of the firmware or electrical subteams, you will be writing code. Hopefully you will be writing code that will adhear to our [style guide](null "found here") which helps ensure your code is readable to other programmers incase you are not there to defend it. Remember, when you write code, you write code for *others* not for yourself. Of course you'll be able to understand your own code, but it's imperative that others can also understand it, be able to edit it, and improve on it.

### README
Every folder that contains a new board should have a toplevel README. It should be pretty short (unless you have a complex board) and explain the overall role of the board. It should cover what the board does, where it is located in the car, and who the [main author](null "please use your github username here with an @ symbol so someone can easily contact you") is.
Additionally, this readme should include a schematic of the board so that someone can consult the shchematic while looking at your code to identify inconsistencies.

### Commenting
**COMMENT YOUR CODE!** No seriously, comment your code. Not only is it horrible codding practice not to, but it makes it incredibly difficult for [someone](null "Even you...") to go back and understand your code.
The code on this repo is public which means that potential employers can check it [out](null "Especially if you pin it to your profile (which you should)") so you want to be demonstrating your best coding practices. Comments should be in the form of:
```C   
    /* Full line */
```
These should be used at the top of functions to explain their purpose and function as well as what the input variables are and what the output is.
```C
    // Single line
```
These should be used to parce variable names and explain their use
```C
    /*--------------------------
    |     Document Headers     |
    |   Author @segerpeter07   |
    --------------------------*/
```
These should be at the top of every document you write and should explain what the document includes and should of course have the author listed. This is your work, make sure you can brag about it!

