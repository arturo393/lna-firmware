# include "UartHandler.hpp"

class Rs232UartHandler : public UartHandler
{
private:
    /* data */
public:
    Rs232UartHandler(/* args */);
    ~Rs232UartHandler();
    public void data_enabler();
    public void data_disabler();
};

Rs232UartHandler::Rs232UartHandler(/* args */)
{
}

Rs232UartHandler::~Rs232UartHandler()
{
}
