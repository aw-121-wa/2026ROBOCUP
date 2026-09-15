#include "host_command.h"
#include <string.h>

static bool space(char c)
{
    return c == ' ' || c == '\t';
}
static int parse(char *line, HostCommand *command)
{
    while (space(*line))
        ++line;
    size_t n = strlen(line);
    while (n && space(line[n - 1]))
        line[--n] = 0;
    if (!n)
        return HOST_IDLE;
    *command = (HostCommand){HOST_NONE, 0};
    if (!strcmp(line, "ARM"))
    {
        command->kind = HOST_ARM;
        return HOST_OK;
    }
    if (!strcmp(line, "PATH"))
    {
        command->kind = HOST_PATH;
        return HOST_OK;
    }
    if (!strcmp(line, "STOP"))
    {
        command->kind = HOST_STOP;
        return HOST_OK;
    }
    char *arg;
    if (!strncmp(line, "FORWARD", 7) && space(line[7]))
    {
        command->kind = HOST_FORWARD;
        arg = line + 7;
    }
    else if (!strncmp(line, "SHIFT", 5) && space(line[5]))
    {
        command->kind = HOST_SHIFT;
        arg = line + 5;
    }
    else
        return HOST_SYNTAX;
    while (space(*arg))
        ++arg;
    float sign = 1;
    if (*arg == '+' || *arg == '-')
    {
        if (*arg == '-')
            sign = -1;
        ++arg;
    }
    bool digit = false, decimal = false, range = false;
    float value = 0, place = 0.1f;
    for (; *arg; ++arg)
    {
        if (*arg == '.' && !decimal)
        {
            decimal = true;
            continue;
        }
        if (*arg < '0' || *arg > '9')
            return HOST_SYNTAX;
        digit = true;
        if (!range)
        {
            if (decimal)
            {
                value += (*arg - '0') * place;
                place *= 0.1f;
            }
            else
                value = value * 10 + (*arg - '0');
            if (value > 5000)
                range = true;
        }
    }
    if (!digit)
        return HOST_SYNTAX;
    if (range || value < 1)
        return HOST_RANGE;
    command->distance_mm = sign * value;
    return HOST_OK;
}
int HostCommand_Feed(HostParser *parser, uint8_t byte, HostCommand *command)
{
    if (byte == '\r' || byte == '\n')
    {
        bool discard = parser->discard;
        parser->line[parser->length] = 0;
        parser->length = 0;
        parser->discard = false;
        return discard ? HOST_SYNTAX : parse(parser->line, command);
    }
    if (parser->discard)
        return HOST_IDLE;
    if ((byte < 32 && byte != '\t') || byte > 126 || parser->length >= sizeof(parser->line) - 1)
    {
        parser->discard = true;
        return HOST_IDLE;
    }
    parser->line[parser->length++] = (char)byte;
    return HOST_IDLE;
}
int HostCommand_Check(const HostCommand *command, bool armed, bool busy)
{
    if (command->kind == HOST_STOP)
        return HOST_OK;
    if (command->kind == HOST_ARM)
        return busy ? HOST_BUSY : HOST_OK;
    if (command->kind != HOST_FORWARD && command->kind != HOST_SHIFT && command->kind != HOST_PATH)
        return HOST_SYNTAX;
    if (!armed)
        return HOST_NOT_READY;
    return busy ? HOST_BUSY : HOST_OK;
}
