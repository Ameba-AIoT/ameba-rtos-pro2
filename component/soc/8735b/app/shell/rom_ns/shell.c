/*  Command Line Interface (Command Shell) for microcontrollers.
    Copyright (C) 2014 Jesus Ruben Santa Anna Zamudio.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Author website: http://www.geekfactory.mx
    Author e-mail: ruben at geekfactory dot mx
 */

/// @cond DOXYGEN_SHELL_ROM_CODE
/**
 * @addtogroup util_shell_rom_func Shell Command ROM APIs.
 * @ingroup util_shell
 * @{
 * @brief Shell Command ROM API. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of Shell Command APIs in the RAM space is provided for the user application.
 */

#include "shell.h"
#include "diag.h"

BOOL shell_is_initialized(shell_command_t *pshell_cmd);
void shell_print_commands(shell_command_t *pshell_cmd);
void shell_print(shell_command_t *pshell_cmd, const char *string);
void shell_println(shell_command_t *pshell_cmd, const char *string);

SECTION_SHELL_RODATA static const char shell_cmd_prompt[] = "$>";

/**
 * Parses the string and finds all the substrings (arguments)
 *
 * @param buf The buffer containing the original string
 *
 * @param argv Pointer to char * array to place the pointers to substrings
 *
 * @param maxargs The maximum number of pointers that the previous array can hold
 *
 * @return The total of args parsed
 */
static int shell_parse(char *buf, char **argv, unsigned short maxargs);

static void shell_unparse(char *buf, unsigned int length);

/**
 *  Prints the command shell prompt
 *
 * @param pshell_cmd Point to the command shell handler structure.
 */
static void shell_prompt(shell_command_t *pshell_cmd);

static void line_backspace(shell_command_t *pshell_cmd);
static void line_add_char(shell_command_t *pshell_cmd, char c);
static void line_delete_char(shell_command_t *pshell_cmd);
static void line_home(shell_command_t *pshell_cmd);
static void line_end(shell_command_t *pshell_cmd);
#if CONFIG_SHELL_HISTORY_EN
static void line_clear(shell_command_t *pshell_cmd);

static void set_line(shell_command_t *pshell_cmd, char *line);
static void history_set_line(shell_command_t *pshell_cmd, int entry);
static int history_previous(shell_command_t *pshell_cmd, int entry);
static void history_up(shell_command_t *pshell_cmd);
static void history_down(shell_command_t *pshell_cmd);
static void history_show(shell_command_t *pshell_cmd);
static int history_delete(shell_command_t *pshell_cmd, int entry, int size);
static void history_add(shell_command_t *pshell_cmd);
#endif
static int process_escape(shell_command_t *pshell_cmd);
static void auto_complete(shell_command_t *pshell_cmd);

#if 0
#define SHELL_CMD_WRITE(c)  do {pshell_cmd->shell_writer(pshell_cmd->adapter, (c));} while(0)
#define SHELL_CMD_READ(c)   pshell_cmd->shell_reader (pshell_cmd->adapter, (c))
#else
#define SHELL_CMD_WRITE(c)  do {pshell_cmd->shell_writer((c));} while(0)
#define SHELL_CMD_READ(c)   pshell_cmd->shell_reader ((c))
#endif

/**
 * @brief Check if this shell command entity is totally initialized and ready to run.
 *
 * @details There some resource should be initialized before we can start to run the shell command:
 *            - The receive buffer, which can be initialed by shell_set_cmd_buf().
 *            - The history buffer, which can be initialized by shell_set_cmd_buf().
 *            - Either the command table or command list buffer. The command table can be initialized by
 *              shell_set_cmd_table(); the command list buffer can be initialized by shell_set_cmd_list().
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @return  Returns TRUE if the shell was succesfully initialized and ready to go, returns FALSE
 *          otherwise.
 */
SECTION_SHELL_TEXT BOOL shell_is_initialized(shell_command_t *pshell_cmd)
{
	if ((pshell_cmd->shell_reader == NULL) || (pshell_cmd->shell_writer == NULL)) {
		return FALSE;
	}

	if ((pshell_cmd->cmd_table == NULL) && (pshell_cmd->cmd_list == NULL)) {
		return FALSE;
	}

	if (pshell_cmd->shellbuf == NULL) {
		return FALSE;
	}

#if CONFIG_SHELL_HISTORY_EN
	if (pshell_cmd->hist_buf == NULL) {
		return FALSE;
	}
#endif

	return TRUE;
}

/**
 * @brief Initialize the shell command line interface.
 *
 * @details Initializes the resources used by the command line interface. This function
 *          also tries to start the command line task if the option is enabled. The main
 *          program for the command line interface where all the data is handled is
 *          shell_task().
 *
 * @param[in]   pshell_cmd  The The shell command entity to be initialized.
 *
 * @param[in]   reader      The callback function used to get a character from the stream.
 *
 * @param[in]   writer      The callback function used to write a character to the stream.
 *
 * @param[in]   adapter     The user pointer for the character read/write callback function calling.
 *                          This pointer usually point to a handler of an UART/terminal device.
 *
 * @return  Returns TRUE if the shell was succesfully initialized, returns FALSE
 *          otherwise.
 */
SECTION_SHELL_TEXT BOOL _shell_init(shell_command_t *pshell_cmd, shell_reader_t reader,
									shell_writer_t writer, void *adapter)
{
	if (reader == NULL || writer == NULL) {
		return FALSE;
	}

	_shell_unregister_all(pshell_cmd);

	pshell_cmd->shell_reader = reader;
	pshell_cmd->shell_writer = writer;

	pshell_cmd->adapter = adapter;

	if (pshell_cmd->prompt == (char *)0) {
		pshell_cmd->prompt = shell_cmd_prompt;
	}
//	shell_prompt(pshell_cmd);

	pshell_cmd->initialized = shell_is_initialized(pshell_cmd);


	return TRUE;
}

/**
 * @brief Assign a buffer for the command characters receiving.
 *        Before we start the shell command task, the command receive buffer MUST be assigned.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   buf         The given buffer.
 *
 * @param[in]   buf_size    The size of the given buffer, in bytes.
 *
 * @return  Returns TRUE if the shell was succesfully initialized, returns FALSE
 *          otherwise.
 */
SECTION_SHELL_TEXT BOOL _shell_set_cmd_buf(shell_command_t *pshell_cmd, char *buf, unsigned int buf_size)
{
	if (buf == 0) {
		return FALSE;
	}

	pshell_cmd->shellbuf = buf;
	pshell_cmd->shellbuf_sz = buf_size - 1; // reserve a char as string end(\0)

	pshell_cmd->initialized = shell_is_initialized(pshell_cmd);

	return TRUE;
}

/**
 * @brief To assign a buffer for the command history.
 *
 * @details This buffer is used to store commands ever entered. So we can re-call a command
 *          from the history buffer. This is a ring buffer. Once the buffer is full, the new
 *          added command will overwrite the oldest command.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   buf         The given buffer.
 *
 * @param[in]   buf_size    The size of the given buffer, in bytes.
 *
 * @return  Returns TRUE if the shell was succesfully initialized, returns FALSE
 *          otherwise.
 */
SECTION_SHELL_TEXT BOOL _shell_set_hist_buf(shell_command_t *pshell_cmd, char *buf, unsigned int buf_size)
{
#if CONFIG_SHELL_HISTORY_EN
	if (buf == 0) {
		return FALSE;
	}

	pshell_cmd->hist_buf = buf;
	pshell_cmd->hist_buf_sz = buf_size - 1; // reserve a char as string end(\0)
	pshell_cmd->initialized = shell_is_initialized(pshell_cmd);
#endif
	return TRUE;
}

/**
 * @brief Set the prompt string of the shell command.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   prompt      The pointer of the prompt string.
 *
 * @return  void.
 */
SECTION_SHELL_TEXT void _shell_set_prompt(shell_command_t *pshell_cmd, const char *prompt)
{
	pshell_cmd->prompt = prompt;

	if (pshell_cmd->initialized) {
		shell_print(pshell_cmd, "\r\n");
		shell_prompt(pshell_cmd);
	}
}

/**
 * @brief Assign a prefixed command table to the shell command.
 *
 * @details When a command string is entered, we will search the match command from this table
 *          and execute the callback function if a matched command is found.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   cmd_table   The start pointer of the prefixed command table.
 *
 */
SECTION_SHELL_TEXT void _shell_set_cmd_table(shell_command_t *pshell_cmd, const shell_command_entry_t *cmd_table)
{
	pshell_cmd->cmd_table = cmd_table;
	pshell_cmd->initialized = shell_is_initialized(pshell_cmd);
}

/**
 * @brief Hooks a registrable command entry buffer to the shell command. A new command entry can be
 *        add to this command list at run time.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   cmd_list    The buffer for the registerable of command entry list.
 *
 * @param[in]   list_size   The command list size, which means the maximum number of commands we can add to this list.
 *
 */
SECTION_SHELL_TEXT void _shell_set_cmd_list(shell_command_t *pshell_cmd, shell_command_entry_t *cmd_list,
		unsigned int list_size)
{
	pshell_cmd->cmd_list = cmd_list;
	pshell_cmd->cmd_list_size = list_size;
	pshell_cmd->initialized = shell_is_initialized(pshell_cmd);
}

/**
 * @brief Register a command entry on the registerable command list.
 *
 * @details Registers a command on the registerable command list. The command string and the
 *          corresponding callback function should be provided.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   program     The callback function for the new register command.
 *
 * @param[in]   cmd_string  The new command to be registered.
 *
 * @param[in]   help_string A describtion about the command to be registered.
 *
 * @return  Returns TRUE if command was successfully added to the command list,
 *          or returns FALSE if something fails (no more commands can be registered).
 */

SECTION_SHELL_TEXT BOOL _shell_register(shell_command_t *pshell_cmd, shell_command_entry_t *entry_cmd)
{
	unsigned char i;

	if (pshell_cmd->cmd_list == NULL) {
		return FALSE;
	}

	for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
		if ((pshell_cmd->cmd_list[i].shell_program != (shell_program_t)0) ||
			(pshell_cmd->cmd_list[i].shell_command_string != (const char *)0)) {
			continue;
		}
		pshell_cmd->cmd_list[i].shell_program = entry_cmd->shell_program;
		pshell_cmd->cmd_list[i].shell_command_string = entry_cmd->shell_command_string;
		pshell_cmd->cmd_list[i].shell_command_type = entry_cmd->shell_command_type;
		pshell_cmd->cmd_list[i].help_string = entry_cmd->help_string;
		return TRUE;
	}
	return FALSE;
}

/**
 * @brief Removes all commands entrys in the registrable command list.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 * @return  void.
 */
SECTION_SHELL_TEXT void _shell_unregister_all(shell_command_t *pshell_cmd)
{
	unsigned char i;

	if (pshell_cmd->cmd_list == NULL) {
		return;
	}

	for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
		pshell_cmd->cmd_list[i].shell_program = (shell_program_t)0;
		pshell_cmd->cmd_list[i].shell_command_string = (const char *)0;
		pshell_cmd->cmd_list[i].help_string = (const char *)0;
	}
}

/**
 * @brief set current type.
 *
 * @details set current type for print on this type command
 *
 *
 * @param[in]   pshell_cmd  The command shell entity.
 * @return  void.
 */
SECTION_SHELL_TEXT void shell_set_current_type(shell_command_t *pshell_cmd)
{
	if ((pshell_cmd->cmd_table != NULL) &&
		((pshell_cmd->argc > 1))) {
		if (shell_strcmp(pshell_cmd->argv_list[1], "..") == 0) {
			shell_memset(pshell_cmd->current_type, 0x0, 32);
		} else {
			shell_memcpy(pshell_cmd->current_type, pshell_cmd->argv_list[1], shell_strlen(pshell_cmd->argv_list[1]));
		}

	}

	//shell_memcpy((pshell_cmd->shellbuf + pshell_cmd->buf_count), line, size);
}

/**
 * @brief Prints all available commands.
 *
 * @details Prints a list of all available commands in the prefixed command table
 *          and the registrable command list table.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 * @return  void.
 */
SECTION_SHELL_TEXT void shell_print_commands(shell_command_t *pshell_cmd)
{
	unsigned int i;

	if ((pshell_cmd->cmd_table != NULL) &&
		((pshell_cmd->argc == 1) ||
		 (shell_strcmp(pshell_cmd->argv_list[1], "rom") == 0))) {
		for (i = 0; pshell_cmd->cmd_table[i].shell_command_string != (const char *)NULL; i++) {
			if (pshell_cmd->cmd_table[i].shell_program != (shell_program_t)0) {
//              shell_print(pshell_cmd, pshell_cmd->cmd_table[i].shell_command_type);
//              shell_print(pshell_cmd, pshell_cmd->cmd_table[i].shell_command_string);
//              shell_print(pshell_cmd, "\t: ");
//              shell_println(pshell_cmd, pshell_cmd->cmd_table[i].help_string);
#if CONFIG_FPGA && (CONFIG_SIMU_ASIC_FLOW_EN == 0) // FPGA  
				_rtl_printf("[%6s]%16s : %s \r\n"\
							, pshell_cmd->cmd_table[i].shell_command_type\
							, pshell_cmd->cmd_table[i].shell_command_string\
							, pshell_cmd->cmd_table[i].help_string);
#else   // PXP & ASIC
				// Consider secure issue
				if (shell_strcmp((pshell_cmd->cmd_table[i].shell_command_type), "ROM") != 0) {
					_rtl_printf("[%6s]%16s : %s \r\n"\
								, pshell_cmd->cmd_table[i].shell_command_type\
								, pshell_cmd->cmd_table[i].shell_command_string\
								, pshell_cmd->cmd_table[i].help_string);
				}
#endif
			}
		}
	}

	if (pshell_cmd->cmd_list != NULL) {
		for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
			if ((pshell_cmd->cmd_list[i].shell_program != (shell_program_t)0) &&
				(pshell_cmd->cmd_list[i].shell_command_string != (const char *)0) &&
				(shell_strcmp(pshell_cmd->current_type, pshell_cmd->cmd_list[i].shell_command_type) == 0 ||
				 *pshell_cmd->current_type == 0x0)) {
#if CONFIG_FPGA && (CONFIG_SIMU_ASIC_FLOW_EN == 0) // FPGA                   
				_rtl_printf("[%6s]%16s : %s \r\n"\
							, pshell_cmd->cmd_list[i].shell_command_type\
							, pshell_cmd->cmd_list[i].shell_command_string\
							, pshell_cmd->cmd_list[i].help_string);
#else   // PXP & ASIC
				// Consider secure issue
				if ((shell_strcmp((pshell_cmd->cmd_list[i].shell_command_type), "BOOT_ROM") != 0) &&
					(shell_strcmp((pshell_cmd->cmd_list[i].shell_command_type), "PG Tool") != 0)) {
					_rtl_printf("[%6s]%16s : %s \r\n"\
								, pshell_cmd->cmd_list[i].shell_command_type\
								, pshell_cmd->cmd_list[i].shell_command_string\
								, pshell_cmd->cmd_list[i].help_string);
				}
#endif
//              shell_print(pshell_cmd, pshell_cmd->cmd_list[i].shell_command_string);
//              shell_print(pshell_cmd, "\t: ");
//              shell_println(pshell_cmd, pshell_cmd->cmd_list[i].help_string);
			}
		}
	}
}

/**
 * @brief Prints a string to the terminal.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   string  The string to send to the terminal
 * @return  void.
 */
SECTION_SHELL_TEXT void shell_print(shell_command_t *pshell_cmd, const char *string)
{
	while (*string != '\0') {
		SHELL_CMD_WRITE(*string++);
	}
}

/**
 * @brief Prints a string and moves the cursor to the next line
 *
 * @details Displays a string on the terminal and moves the cursor to the next line.
 *          The string should be null terminated.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @param[in]   string      The string to send to the terminal.
 * @return  void.
 */
SECTION_SHELL_TEXT void shell_println(shell_command_t *pshell_cmd, const char *string)
{
	shell_print(pshell_cmd, string);
	shell_print(pshell_cmd, (const char *)"\r\n");
}

/**
 * @brief Converts a string to an all lower case string.
 *
 * @param[in]   s  The string to be converted.
 *
 * @param[in]   d  The buffer for the converted string.
 * @return  void.
 */
SECTION_SHELL_TEXT void shell_string_to_lower_case(char *s, char *d)
{
	while (*s != '\0') {
		if ((*s >= 0x41) && (*s <= 0x5A)) {
			*d = *s + 0x20;
		} else {
			*d = *s;
		}
		s++;
		d++;
	}
	*d = '\0';
}

/**
 * @brief Main shell command processing loop.
 *
 * @details This function implements the main functionality of the shell command line interface.
 *          This function should be called frequently so it can handle the input from the terminal
 *          data stream.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @return  -1: if the shell command entity is not initialed.
 * @return   0: wait for a command input or no match command was found for the input string.
 * @return   other: the return value from the command handler function.
 */
SECTION_SHELL_TEXT s32 _shell_task(shell_command_t *pshell_cmd)
{
	unsigned int i = 0, retval = 0;
//	int argc = 0;
	char rxchar = 0;
	char finished = 0;
	char cmd_match;

	if (!pshell_cmd->initialized) {
		return -1;
	}

	// Process each one of the received characters
	if (SHELL_CMD_READ(&rxchar)) {
		if (pshell_cmd->escape_len) {
			pshell_cmd->escape[pshell_cmd->escape_len++] = rxchar;
			if (process_escape(pshell_cmd)) {
				pshell_cmd->escape_len = 0;
			} else if (pshell_cmd->escape_len == CONFIG_SHELL_MAX_ESCAPE_LEN) {
				/* Not a valid escape sequence, and buffer full. */
				pshell_cmd->escape_len = 0;
			}
			return 0;
		}

		switch (rxchar) {
		case SHELL_ASCII_ESC: // For VT100 escape sequences
			/* Start of escape sequence. */
			pshell_cmd->escape[pshell_cmd->escape_len++] = rxchar;
			break;

		case SHELL_ASCII_HT:
#if CONFIG_SHELL_AUTO_COMP_EN
			auto_complete(pshell_cmd);
#endif
			break;

		case SHELL_ASCII_CR: // Enter key pressed
		case SHELL_ASCII_LF:
			pshell_cmd->shellbuf[pshell_cmd->buf_count] = '\0';
			shell_println(pshell_cmd, "");
			finished = 1;
			break;

		//case SHELL_ASCII_DEL:
		case SHELL_ASCII_BS: // Backspace pressed, Ctrl+H means ascii 0x08
			if (pshell_cmd->buf_count > 0) {
				line_backspace(pshell_cmd);
			}
			break;

		case SHELL_ASCII_DEL: // Delete pressed, assci same as backspace button is 0x7F
			line_delete_char(pshell_cmd);
			break;

		default:
			// Process printable characters, but ignore other ASCII chars
			if ((pshell_cmd->buf_count < pshell_cmd->shellbuf_sz) &&
				(rxchar >= 0x20) &&
				(rxchar < 0x7F)) {
				line_add_char(pshell_cmd, rxchar);
			}
		}

		// Check if a full command is available on the buffer to process
		if (finished) {
			if (pshell_cmd->buf_count == 0) {
				shell_prompt(pshell_cmd);
				return 0;
			}
#if CONFIG_SHELL_HISTORY_EN
			if (pshell_cmd->buf_count != 0) {
				history_add(pshell_cmd);
			}
#endif
			pshell_cmd->argc = shell_parse(pshell_cmd->shellbuf, pshell_cmd->argv_list, CONFIG_SHELL_MAX_COMMAND_ARGS);

			if (!shell_strcmp(pshell_cmd->argv_list[0], "cd")) {
				shell_set_current_type(pshell_cmd);

				finished = 0;
			}

			if (!shell_strcmp(pshell_cmd->argv_list[0], "ls")) {
				shell_print_commands(pshell_cmd);

				finished = 0;
			}


			if (!shell_strcmp(pshell_cmd->argv_list[0], "help") ||
				!shell_strcmp(pshell_cmd->argv_list[0], "?")) {
				shell_print_commands(pshell_cmd);

				// Run the appropiate function
				//shell_print_commands (pshell_cmd);
				finished = 0;
			}

#if CONFIG_SHELL_HISTORY_EN
			if (!shell_strcmp(pshell_cmd->argv_list[0], "history")) {
				// Run the appropiate function
				history_show(pshell_cmd);
				finished = 0;
			}
#endif
			// sequential search on command table
			if (finished != 0 && pshell_cmd->buf_count != 0) {
				if (pshell_cmd->cmd_table != (shell_command_entry_t *)NULL) {
					if (pshell_cmd->case_conv_buf != NULL) {
						shell_string_to_lower_case(pshell_cmd->argv_list[0], pshell_cmd->argv_list[0]);
					}

					for (i = 0; pshell_cmd->cmd_table[i].shell_command_string != NULL; i++) {
						if (pshell_cmd->cmd_table[i].shell_program == NULL) {
							continue;
						}
						cmd_match = 0;


						// If string matches one on the list
						if (pshell_cmd->case_conv_buf != NULL) {
							shell_string_to_lower_case((char *)(pshell_cmd->cmd_table[i].shell_command_string),
													   pshell_cmd->case_conv_buf);
							if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->case_conv_buf)) {
								cmd_match = 1;
							}

						} else {
							if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_table[i].shell_command_string)) {
								cmd_match = 1;
							}
						}

						if (cmd_match) {
							// Run the appropiate function
							retval = pshell_cmd->cmd_table[i].shell_program((pshell_cmd->argc - 1), &pshell_cmd->argv_list[1]);
							finished = 0;
							break;  // break for loop
						}
					}
				}
			}

			if (finished != 0 && pshell_cmd->buf_count != 0) {
				if (pshell_cmd->cmd_list != (shell_command_entry_t *)NULL) {
					if (pshell_cmd->case_conv_buf != NULL) {
						shell_string_to_lower_case(pshell_cmd->argv_list[0], pshell_cmd->argv_list[0]);
					}

					for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
						if ((pshell_cmd->cmd_list[i].shell_program == NULL) ||
							(pshell_cmd->cmd_list[i].shell_command_string == NULL)) {
							continue;
						}
						cmd_match = 0;
						// If string matches one on the list
						if (pshell_cmd->case_conv_buf != NULL) {
							shell_string_to_lower_case((char *)(pshell_cmd->cmd_list[i].shell_command_string),
													   pshell_cmd->case_conv_buf);
							if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->case_conv_buf)) {
								cmd_match = 1;
							}

						} else {
							if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_list[i].shell_command_string)) {
								cmd_match = 1;
							}
						}

						if (cmd_match) {
							// Run the appropiate function
							retval = pshell_cmd->cmd_list[i].shell_program((pshell_cmd->argc - 1), &pshell_cmd->argv_list[1]);
							finished = 0;
							break;  // break the for loop
						}
					}
				}
			}

			if (finished != 0 && pshell_cmd->buf_count != 0) {
				// If no command found and buffer not empty
				shell_println(pshell_cmd, (const char *)"Command NOT found."); // Print not found!!
			}

			pshell_cmd->buf_count = 0;
			pshell_cmd->shellbuf[0] = 0;
			pshell_cmd->buf_pos = 0;
			pshell_cmd->hist_step = -1;
			pshell_cmd->escape_len = 0;
//			shell_println(pshell_cmd, "");
			shell_prompt(pshell_cmd);
		}
	}

	return retval;
}

/*-------------------------------------------------------------*/
/*      Internal functions              */
/*-------------------------------------------------------------*/
SECTION_SHELL_TEXT static int shell_parse(char *buf, char **argv, unsigned short maxargs)
{
	uint32_t i = 0;
	uint32_t argc = 0;
	uint32_t length = shell_strlen(buf) + 1; //String lenght to parse = strlen + 1
	char toggle = 0;


	argv[argc] = &buf[0];

	for (i = 0; i < length && argc < maxargs; i++) {
		switch (buf[i]) {
		// String terminator means at least one arg
		case '\0':
			i = length;
			argc++;
			break;

		// Check for double quotes for strings as parameters
		case '\"':
			if (toggle == 0) {
				toggle = 1;
				buf[i] = '\0';
				argv[argc] = &buf[i + 1];
			} else {
				toggle = 0;
				buf[i] = '\0';

			}
			break;

		case ' ':
			if (toggle == 0) {
				buf[i] = '\0';
				argc++;
				argv[argc] = &buf[i + 1];
			}
			break;

		}
	}

	return argc;
}

SECTION_SHELL_TEXT static void shell_unparse(char *buf, unsigned int length)
{
	int quoted;
	uint32_t i;

	quoted = FALSE;
	for (i = 0; i < length; i++) {
		if (buf[i] == '"') {
			quoted = TRUE;
			continue;
		}

		if (!buf[i]) {
			if (quoted) {
				buf[i] = '"';
				quoted = FALSE;
			} else {
				buf[i] = ' ';
			}
		}
	}
	buf[length] = 0;
}

SECTION_SHELL_TEXT static void shell_prompt(shell_command_t *pshell_cmd)
{
	shell_print(pshell_cmd, pshell_cmd->prompt);
}

SECTION_SHELL_TEXT static void line_backspace(shell_command_t *pshell_cmd)
{
	uint32_t i;
	if (pshell_cmd->buf_pos == 0) {
		return;
	} else {
		if (pshell_cmd->buf_pos == pshell_cmd->buf_count) {
			pshell_cmd->shellbuf[pshell_cmd->buf_count - 1] = 0;
			//shell_print (pshell_cmd, "\x1b\x5b\x43 \x1b\x5b\x43");
			shell_print(pshell_cmd, "\x1b\x5b\x44");
			SHELL_CMD_WRITE(0x20);
			shell_print(pshell_cmd, "\x1b\x5b\x44");

		} else {
			shell_memmove((pshell_cmd->shellbuf + pshell_cmd->buf_pos - 1),
						  (pshell_cmd->shellbuf + pshell_cmd->buf_pos),
						  (pshell_cmd->buf_count - pshell_cmd->buf_pos + 1));
			shell_print(pshell_cmd, "\x1b\x5b\x44");
			shell_print(pshell_cmd, (pshell_cmd->shellbuf + pshell_cmd->buf_pos - 1));
			SHELL_CMD_WRITE(0x20);

			// move cousor left to the position
			for (i = 0; i < (pshell_cmd->buf_count - pshell_cmd->buf_pos + 1); i++) {
				shell_print(pshell_cmd, "\x1b\x5b\x44");
			}

		}
		pshell_cmd->buf_count--;
		pshell_cmd->buf_pos--;
	}
}

SECTION_SHELL_TEXT static void line_add_char(shell_command_t *pshell_cmd, char c)
{
	unsigned int i;

	if (pshell_cmd->buf_pos == pshell_cmd->buf_count) {
		pshell_cmd->shellbuf[pshell_cmd->buf_count] = c;
		pshell_cmd->buf_count++;
		pshell_cmd->shellbuf[pshell_cmd->buf_count] = 0;
		SHELL_CMD_WRITE(c);
		pshell_cmd->buf_pos++;
	} else {
		shell_memmove((pshell_cmd->shellbuf + pshell_cmd->buf_pos + 1),
					  (pshell_cmd->shellbuf + pshell_cmd->buf_pos),
					  (pshell_cmd->buf_count - pshell_cmd->buf_pos + 1));
		pshell_cmd->shellbuf[pshell_cmd->buf_pos] = c;
		shell_print(pshell_cmd, (pshell_cmd->shellbuf + pshell_cmd->buf_pos));
		for (i = 0; i < (pshell_cmd->buf_count - pshell_cmd->buf_pos); i++) {
			shell_print(pshell_cmd, "\x1b\x5b\x44");
		}
		pshell_cmd->buf_count++;
		pshell_cmd->buf_pos++;
	}

}

SECTION_SHELL_TEXT static void line_delete_char(shell_command_t *pshell_cmd)
{
	int i;

	if (pshell_cmd->buf_pos < pshell_cmd->buf_count) {
		shell_memmove((pshell_cmd->shellbuf + pshell_cmd->buf_pos),
					  (pshell_cmd->shellbuf + pshell_cmd->buf_pos + 1),
					  (pshell_cmd->buf_count - pshell_cmd->buf_pos));
		shell_print(pshell_cmd, (pshell_cmd->shellbuf + pshell_cmd->buf_pos));
		SHELL_CMD_WRITE(0x20);
		for (i = 0; i < (pshell_cmd->buf_count - pshell_cmd->buf_pos); i++) {
			shell_print(pshell_cmd, "\x1b\x5b\x44");
		}
		pshell_cmd->buf_count--;
	}
}
#if CONFIG_SHELL_HISTORY_EN

SECTION_SHELL_TEXT static void line_clear(shell_command_t *pshell_cmd)
{
	// move cousor to the end
	while (pshell_cmd->buf_pos < pshell_cmd->buf_count) {
		pshell_cmd->buf_pos++;
		// move cousor right
		shell_print(pshell_cmd, "\x1b\x5b\x43");
	}

	// do backspace to clear all character
	while (pshell_cmd->buf_pos) {
		line_backspace(pshell_cmd);
	}
}
#endif
SECTION_SHELL_TEXT static void line_home(shell_command_t *pshell_cmd)
{
	while (pshell_cmd->buf_pos) {
		pshell_cmd->buf_pos--;
		shell_print(pshell_cmd, "\x1b\x5b\x44");
	}
}

SECTION_SHELL_TEXT static void line_end(shell_command_t *pshell_cmd)
{
	while (pshell_cmd->buf_pos < pshell_cmd->buf_count) {
		pshell_cmd->buf_pos++;
		shell_print(pshell_cmd, "\x1b\x5b\x43");
	}
}

#if CONFIG_SHELL_HISTORY_EN
// Set receive buffer from a given string (from the history buffer)
SECTION_SHELL_TEXT static void set_line(shell_command_t *pshell_cmd, char *line)
{
	uint32_t size;

	size = shell_strlen(line);
	if (size > pshell_cmd->shellbuf_sz) {
//        line_add_char(pshell_cmd, '!');
		return;
	}

	if (pshell_cmd->buf_pos == pshell_cmd->buf_count) {
		shell_print(pshell_cmd, line);
		pshell_cmd->buf_pos += size;
		shell_memcpy((pshell_cmd->shellbuf + pshell_cmd->buf_count), line, size);
		pshell_cmd->buf_count += size;
		pshell_cmd->shellbuf[pshell_cmd->buf_count] = 0;
	}
}

SECTION_SHELL_TEXT static void history_set_line(shell_command_t *pshell_cmd, int entry)
{
	int i;

	set_line(pshell_cmd, pshell_cmd->hist_buf + entry);
	pshell_cmd->hist_step = entry;

	if (entry > 0) {
		/* Did we drop off the end of the buffer? */
		for (i = entry; i < pshell_cmd->hist_buf_sz; i++) {
			if (!pshell_cmd->hist_buf[i]) {
				break;
			}
		}

		if (i == pshell_cmd->hist_buf_sz) {
			/* Yes we did */
			set_line(pshell_cmd, pshell_cmd->hist_buf);
		}
	}
}

// search previous command string start from the history buffer
SECTION_SHELL_TEXT static int history_previous(shell_command_t *pshell_cmd, int entry)
{
	int prev;

	if (entry == pshell_cmd->hist_begin) {
		return -1;
	}

	/* Onto the terminating zero of the previous entry. */
	if (--entry < 0) {
		entry = pshell_cmd->hist_buf_sz - 1;
	}

	/* Onto the last byte of the previous entry. */
	prev = entry;
	if (--entry < 0) {
		entry = pshell_cmd->hist_buf_sz - 1;
	}

	// search previous string end '\0'
	while (pshell_cmd->hist_buf[entry]) {
		prev = entry;
		if (--entry < 0) {
			entry = pshell_cmd->hist_buf_sz - 1;
		}
	}

	return prev;
}

// Step to previous command string in the history buffer
SECTION_SHELL_TEXT static void history_up(shell_command_t *pshell_cmd)
{
	int entry;

	if (pshell_cmd->hist_step == -1) {
		entry = history_previous(pshell_cmd, pshell_cmd->hist_end);
	} else {
		entry = history_previous(pshell_cmd, pshell_cmd->hist_step);
	}

	if (entry == -1) {
		return;
	}
	line_clear(pshell_cmd);
	history_set_line(pshell_cmd, entry);
	pshell_cmd->hist_step = entry;
}

// Step to the next command string in the history buffer
SECTION_SHELL_TEXT static void history_down(shell_command_t *pshell_cmd)
{
	int i;

	if (pshell_cmd->hist_step == -1) {
		return;
	}

	line_clear(pshell_cmd);
	if (pshell_cmd->hist_step == pshell_cmd->hist_end) {
		pshell_cmd->hist_step = -1;
		return;
	}

	i = pshell_cmd->hist_step;
	while (pshell_cmd->hist_buf[i]) {
		if (++i == pshell_cmd->hist_buf_sz) {
			i = 0;
		}
	}

	if (++i == pshell_cmd->hist_buf_sz) {
		i = 0;
	}

	history_set_line(pshell_cmd, i);
	pshell_cmd->hist_step = i;
}

SECTION_SHELL_TEXT static void history_show(shell_command_t *pshell_cmd)
{
	int entry;
	int i;

	/* Skip the 'history' command itself. */
	entry = history_previous(pshell_cmd, pshell_cmd->hist_end);
	for (entry = history_previous(pshell_cmd, entry); entry != -1;
		 entry = history_previous(pshell_cmd, entry)) {
		shell_print(pshell_cmd, pshell_cmd->hist_buf + entry);

		/* Did we drop off the end of the buffer? */
		for (i = entry; i < pshell_cmd->hist_buf_sz; i++) {
			if (!pshell_cmd->hist_buf[i]) {
				break;
			}
		}

		if (i == pshell_cmd->hist_buf_sz) {
			/* Yes we did */
			shell_print(pshell_cmd, pshell_cmd->hist_buf);
		}

		shell_println(pshell_cmd, "");
	}
}

/* Free at least size, but also fully zero out any deleted entry. */
SECTION_SHELL_TEXT static int history_delete(shell_command_t *pshell_cmd, int entry, int size)
{
	int freed;
	int i;

	freed = 0;
	i = entry;
	while (freed < size || pshell_cmd->hist_buf[i]) {
		pshell_cmd->hist_buf[i] = 0;
		freed++;
		if (++i == pshell_cmd->hist_buf_sz) {
			i = 0;
		}
	}

	pshell_cmd->hist_buf[i] = 0;
	/*
	 * Skip at least over the terminating zero, and any after that,
	 * to the next entry.
	 */
	do {
		if (++i == pshell_cmd->hist_buf_sz) {
			i = 0;
		}
	} while (!pshell_cmd->hist_buf[i] && (i < entry));

	return i;
}

SECTION_SHELL_TEXT static void history_add(shell_command_t *pshell_cmd)
{
	uint32_t size;
	int entry;
	int part1;

	size = shell_strlen(pshell_cmd->shellbuf) + 1;
	if (pshell_cmd->hist_begin == pshell_cmd->hist_end) {
		/* First entry, both are 0. */
		shell_memcpy(pshell_cmd->hist_buf, pshell_cmd->shellbuf, size);
	} else if (pshell_cmd->hist_begin < pshell_cmd->hist_end) {
		if (pshell_cmd->hist_buf_sz - pshell_cmd->hist_end >= size) {
			shell_memcpy(pshell_cmd->hist_buf + pshell_cmd->hist_end, pshell_cmd->shellbuf, size);
		} else {
			part1 = pshell_cmd->hist_buf_sz - pshell_cmd->hist_end;
			entry = pshell_cmd->hist_begin;
			pshell_cmd->hist_begin = history_delete(pshell_cmd, entry, size - part1);
			shell_memcpy(pshell_cmd->hist_buf + pshell_cmd->hist_end, pshell_cmd->shellbuf, part1);
			shell_memcpy(pshell_cmd->hist_buf, pshell_cmd->shellbuf + part1, size - part1);
		}
	} else {
		part1 = pshell_cmd->hist_begin - pshell_cmd->hist_end;
		if (part1 <= size) {
			/* Not enough room between end and begin. */
			pshell_cmd->hist_begin = history_delete(pshell_cmd, pshell_cmd->hist_begin, size - part1);
			part1 = pshell_cmd->hist_buf_sz - pshell_cmd->hist_end;
			if (part1 < size) {
				shell_memcpy(pshell_cmd->hist_buf + pshell_cmd->hist_end, pshell_cmd->shellbuf, part1);
				shell_memcpy(pshell_cmd->hist_buf, pshell_cmd->shellbuf + part1, size - part1);
			} else {
				shell_memcpy(pshell_cmd->hist_buf + pshell_cmd->hist_end, pshell_cmd->shellbuf, size);
			}
		} else {
			/* Enough room between end and begin. */
			shell_memcpy(pshell_cmd->hist_buf + pshell_cmd->hist_end, pshell_cmd->shellbuf, size);
		}
	}

	pshell_cmd->hist_end += size;
	if (pshell_cmd->hist_end >= pshell_cmd->hist_buf_sz) {
		pshell_cmd->hist_end -= pshell_cmd->hist_buf_sz;
	}

	if (pshell_cmd->hist_begin == pshell_cmd->hist_end) {
		pshell_cmd->hist_begin = history_delete(pshell_cmd, pshell_cmd->hist_begin,
												shell_strlen(pshell_cmd->hist_buf + pshell_cmd->hist_begin));
	}
}

#endif      // end of "#if CONFIG_SHELL_HISTORY_EN"

SECTION_SHELL_TEXT static int process_escape(shell_command_t *pshell_cmd)
{
	if (pshell_cmd->escape_len == 4) {
		if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x33\x7e", 4)) {
			/* Delete */
			line_delete_char(pshell_cmd);
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x31\x7e", 4)) {
			/* Home */
			line_home(pshell_cmd);
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x34\x7e", 4)) {
			/* End */
			line_end(pshell_cmd);
		}
	} else if (pshell_cmd->escape_len == 3) {
		if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x41", 3)) {
			/* Up arrow */
#if CONFIG_SHELL_HISTORY_EN
			history_up(pshell_cmd);
#endif
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x42", 3)) {
			/* Down arrow */
#if CONFIG_SHELL_HISTORY_EN
			history_down(pshell_cmd);
#endif
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x44", 3)) {
			/* Left arrow */
			if (pshell_cmd->buf_pos > 0) {
				pshell_cmd->buf_pos--;
				shell_print(pshell_cmd, "\x1b\x5b\x31\x44");
			}
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x5b\x43", 3)) {
			/* Right arrow */
			if (pshell_cmd->buf_pos < pshell_cmd->buf_count) {
				pshell_cmd->buf_pos++;
				shell_print(pshell_cmd, "\x1b\x5b\x31\x43");
			}
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x4f\x48", 3)) {
			/* Home */
			line_home(pshell_cmd);
		} else if (!shell_strncmp(pshell_cmd->escape, "\x1b\x4f\x46", 3)) {
			/* End */
			line_end(pshell_cmd);
		} else {
			return FALSE;
		}
	} else {
		return FALSE;
	}

	return TRUE;
}

#if CONFIG_SHELL_AUTO_COMP_EN
SECTION_SHELL_TEXT static void auto_complete(shell_command_t *pshell_cmd)
{
	shell_command_entry_t *like_cmd;
	int matched;
	unsigned int i;
	int reprompt;
	int argc;
	int cmd_match;

	reprompt = FALSE;
	if (!pshell_cmd->buf_count) {
		/* Tab on an empty line: show all commmands. */
		shell_println(pshell_cmd, "");
		shell_print_commands(pshell_cmd);
		reprompt = TRUE;
//    } else if (pshell_cmd->shellbuf[pshell_cmd->buf_count - 1] != ' ') {
	} else {
		line_end(pshell_cmd);
		/* Try to complete the current word. */
		argc = shell_parse(pshell_cmd->shellbuf, pshell_cmd->argv_list, CONFIG_SHELL_MAX_COMMAND_ARGS);
		if (0 == argc) {
			return;
		}

		if (pshell_cmd->case_conv_buf != NULL) {
			shell_string_to_lower_case(pshell_cmd->argv_list[0], pshell_cmd->argv_list[0]);
		}

		like_cmd = (shell_command_entry_t *) NULL;
		matched = 0;

		if (pshell_cmd->cmd_table != (shell_command_entry_t *)NULL) {
			for (i = 0; pshell_cmd->cmd_table[i].shell_command_string != NULL; i++) {
				if (pshell_cmd->cmd_table[i].shell_program == NULL) {
					continue;
				}

				// If string matches one on the list
				cmd_match = 0;
				if (pshell_cmd->case_conv_buf != NULL) {
					shell_string_to_lower_case((char *)(pshell_cmd->cmd_table[i].shell_command_string),
											   pshell_cmd->case_conv_buf);
					if (!shell_strncmp(pshell_cmd->argv_list[0], pshell_cmd->case_conv_buf,
									   shell_strlen(pshell_cmd->argv_list[0]))) {
						cmd_match = 1;
					}

				} else {
					if (!shell_strncmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_table[i].shell_command_string,
									   shell_strlen(pshell_cmd->argv_list[0]))) {
						cmd_match = 1;
					}
				}

				if (cmd_match) {
					matched++;
					if (like_cmd != (shell_command_entry_t *)NULL) {
						// more than 1 like command, print previous like command
						shell_print(pshell_cmd, "\r\n");
						shell_print(pshell_cmd, like_cmd->shell_command_string);
						if (like_cmd->help_string != NULL) {
							shell_print(pshell_cmd, "\r\n");
							shell_println(pshell_cmd, like_cmd->help_string);
						}
					}
					like_cmd = (shell_command_entry_t *)&pshell_cmd->cmd_table[i];
				}
			}
		}

		if (pshell_cmd->cmd_list != (shell_command_entry_t *)NULL) {
			for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
				if ((pshell_cmd->cmd_list[i].shell_program == NULL) ||
					(pshell_cmd->cmd_list[i].shell_command_string == NULL)) {
					continue;
				}

				// If string matches one on the list
				cmd_match = 0;
				if (pshell_cmd->case_conv_buf != NULL) {
					shell_string_to_lower_case((char *)(pshell_cmd->cmd_list[i].shell_command_string),
											   pshell_cmd->case_conv_buf);
					if (!shell_strncmp(pshell_cmd->argv_list[0],  pshell_cmd->case_conv_buf,
									   shell_strlen(pshell_cmd->argv_list[0]))) {
						cmd_match = 1;
					}

				} else {
					if (!shell_strncmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_list[i].shell_command_string,
									   shell_strlen(pshell_cmd->argv_list[0]))) {
						cmd_match = 1;
					}
				}

				if (cmd_match) {
					matched++;
					if (like_cmd != (shell_command_entry_t *)NULL) {
						// more than 1 like command, print previous like command
						shell_print(pshell_cmd, "\r\n");
						shell_print(pshell_cmd, like_cmd->shell_command_string);
						if (like_cmd->help_string != NULL) {
							shell_print(pshell_cmd, "\r\n");
							shell_println(pshell_cmd, like_cmd->help_string);
						}
					}
					like_cmd = &pshell_cmd->cmd_list[i];
				}
			}
		}

		if (matched > 0) {
			if ((matched > 1) || (argc > 1) ||
				(shell_strlen(pshell_cmd->argv_list[0]) == shell_strlen(like_cmd->shell_command_string))) {
				shell_print(pshell_cmd, "\r\n");
				shell_print(pshell_cmd, like_cmd->shell_command_string);
				if (like_cmd->help_string != NULL) {
					shell_print(pshell_cmd, "\r\n");
					shell_println(pshell_cmd, like_cmd->help_string);
				}
				reprompt = TRUE;
			} else {
				for (i = shell_strlen(pshell_cmd->argv_list[0]); i < shell_strlen(like_cmd->shell_command_string); i++) {
					line_add_char(pshell_cmd, like_cmd->shell_command_string[i]);
				}
				line_add_char(pshell_cmd, ' ');
			}
		}
	}

	// recovery receive buffer, parse will convert space to 0
	shell_unparse(pshell_cmd->shellbuf, pshell_cmd->buf_count);
	if (reprompt) {
		shell_print(pshell_cmd, "\r\n");
		shell_prompt(pshell_cmd);
		if (pshell_cmd->buf_count) {
			shell_print(pshell_cmd, pshell_cmd->shellbuf);
		}
	}
}
#endif // end of "#if CONFIG_SHELL_AUTO_COMP_EN"

/**
 * @brief Receives a line of command and process it.
 * @details This function is similar to the shell_task() but it just only receives and process a command
 *          and then return.
 *
 * @param[in]   pshell_cmd  The command shell entity.
 *
 * @return  -1: if the shell command entity is not initialed.
 * @return   0: no match command was found for the input string.
 * @return   other: the return value from the command handler function.
 */
SECTION_SHELL_TEXT s32 _shell_parse_one_cmd(shell_command_t *pshell_cmd)
{
	uint32_t i = 0;
	int32_t retval = 0;
	int argc = 0;
	char rxchar = 0;
	char finished = 0;

	if (!pshell_cmd->initialized) {
		return -1;
	}

//    dbg_printf("-->1\r\n");
	while (!finished) {
		// Process each one of the received characters
		if (SHELL_CMD_READ(&rxchar)) {
			switch (rxchar) {

			case SHELL_ASCII_CR: // Enter key pressed
			case SHELL_ASCII_LF:
				pshell_cmd->shellbuf[pshell_cmd->buf_count] = '\0';
				finished = 1;
				break;

			default:
				// Process printable characters, but ignore other ASCII chars
				if ((pshell_cmd->buf_count < pshell_cmd->shellbuf_sz) &&
					(rxchar >= 0x20) &&
					(rxchar < 0x7F)) {
					pshell_cmd->shellbuf[pshell_cmd->buf_count] = rxchar;
					pshell_cmd->buf_count++;
					pshell_cmd->shellbuf[pshell_cmd->buf_count] = 0;
					pshell_cmd->buf_pos++;
				}
			}

		}
	}
//    dbg_printf("-->2\r\n");

	// Check if a full command is available on the buffer to process
	if (pshell_cmd->buf_count == 0) {
		return 0;
	}
//    dbg_printf("-->%s\r\n", pshell_cmd->shellbuf);
	argc = shell_parse(pshell_cmd->shellbuf, pshell_cmd->argv_list, CONFIG_SHELL_MAX_COMMAND_ARGS);

	// sequential search on command table
	if (finished != 0 && pshell_cmd->buf_count != 0) {
		if (pshell_cmd->cmd_table != (shell_command_entry_t *)NULL) {
			for (i = 0; pshell_cmd->cmd_table[i].shell_command_string != NULL; i++) {
				if (pshell_cmd->cmd_table[i].shell_program == NULL) {
					continue;
				}
				// If string matches one on the list
				if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_table[i].shell_command_string)) {
					// Run the appropiate function
					retval = pshell_cmd->cmd_table[i].shell_program((argc - 1), &pshell_cmd->argv_list[1]);
					finished = 0;
				}
			}
		}
	}

	if (finished != 0 && pshell_cmd->buf_count != 0) {
		if (pshell_cmd->cmd_list != (shell_command_entry_t *)NULL) {
			for (i = 0; i < pshell_cmd->cmd_list_size; i++) {
				if ((pshell_cmd->cmd_list[i].shell_program == NULL) ||
					(pshell_cmd->cmd_list[i].shell_command_string == NULL)) {
					continue;
				}
				// If string matches one on the list
				if (!shell_strcmp(pshell_cmd->argv_list[0], pshell_cmd->cmd_list[i].shell_command_string)) {
					// Run the appropiate function
					retval = pshell_cmd->cmd_list[i].shell_program((argc - 1), &pshell_cmd->argv_list[1]);
					finished = 0;
				}
			}
		}
	}

	pshell_cmd->buf_count = 0;
	pshell_cmd->shellbuf[0] = 0;
	pshell_cmd->buf_pos = 0;
	pshell_cmd->hist_step = -1;
	pshell_cmd->escape_len = 0;

	return retval;

}

/** @} */ /* End of group util_shell_rom_func */

/// @endcond /* End of condition DOXYGEN_SHELL_ROM_CODE */

