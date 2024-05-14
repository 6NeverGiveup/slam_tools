// \033[0m: 重置所有样式和颜色为默认值。通常用于恢复到终端的默认文本显示属性。
// \033[1m: 设置为加粗样式。加粗的文本通常会显得更加突出。
// \033[2m: 设置为暗淡样式。在某些终端中可能与正常文本样式相同，具体效果因终端而异。
// \033[3m: 设置为斜体样式。但在许多终端中，并不真正支持斜体显示。
// \033[4m: 设置为下划线样式。将文本下方画一条线。
// \033[5m: 设置为闪烁样式。使文本闪烁显示，但在大多数终端中已经不常见了。
// \033[7m: 设置为反显样式。使文本背景色变为前景色，前景色变为背景色，以产生反色效果。
// \033[8m: 设置为隐藏样式。使文本变为透明，通常用于隐藏密码输入等场景。
// \033[9m: 设置为删除线样式。在一些终端中，删除线效果可以通过这种方式实现。
// \033[30m 到 \033[37m: 设置文本颜色为黑色到白色。分别代表不同的颜色，例如\033[31m表示设置文本颜色为红色。具体颜色如下显示
// \033[40m 到 \033[47m: 设置文本背景色为黑色到白色。分别代表不同的背景颜色，例如\033[41m表示设置文本背景色为红色。

#include <iostream>

// ANSI颜色转义码
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define ITALIC  "\033[3m"       /* Italic */
#define UNDERLINE "\033[4m"     /* Underline */

int main() {
    // 打印不同样式的文本
    std::cout << BOLD << "加粗文本" << RESET << std::endl;
    std::cout << ITALIC << "斜体文本" << RESET << std::endl;
    std::cout << UNDERLINE << "下划线文本" << RESET << std::endl;
    
    // 打印带颜色和样式的文本
    std::cout << RED << BOLD << "红色加粗文本" << RESET << std::endl;
    std::cout << BLUE << ITALIC << "蓝色斜体文本" << RESET << std::endl;
    std::cout << GREEN << UNDERLINE << "绿色下划线文本" << RESET << std::endl;
    
    return 0;
}