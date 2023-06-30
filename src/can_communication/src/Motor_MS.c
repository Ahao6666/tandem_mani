#include <stdio.h>

// Define motor states
typedef enum {
    STOPPED,
    STARTUP,
    RUNNING,
    BRAKING,
    ERROR
} MotorState;

// Define motor state transition function
MotorState transition(MotorState currentState, int command) {
    switch (currentState) {
        case STOPPED:
            if (command == 1) {
                return STARTUP;
            }
            break;
        case STARTUP:
            if (command == 2) {
                return RUNNING;
            }
            break;
        case RUNNING:
            if (command == 3) {
                return STOPPED;
            } else if (command == 4) {
                return BRAKING;
            } else if (command == 5) {
                return ERROR;
            }
            break;
        case BRAKING:
            if (command == 3) {
                return STOPPED;
            } else if (command == 5) {
                return ERROR;
            }
            break;
        case ERROR:
            if (command == 2) {
                return STARTUP;
            } else if (command == 4) {
                return RUNNING;
            } else if (command == 3) {
                return STOPPED;
            } else if (command == 5) {
                return ERROR;
            }
            break;
    }
    
    return currentState; // Keep the current state if no matching transition condition
}

// Example usage
int main(int argc, char** argv){
    MotorState currentState = STOPPED;
    int command;
    
    printf("Please enter a command:\n");
    
    while (1) {
        scanf(" %d", &command); // Read the command from the keyboard input
        
        // Simulate state transition
        currentState = transition(currentState, command);
        
        // Print the current state
        switch (currentState) {
            case STOPPED:
                printf("Current state: STOPPED\n");
                break;
            case STARTUP:
                printf("Current state: STARTUP\n");
                break;
            case RUNNING:
                printf("Current state: RUNNING\n");
                break;
            case BRAKING:
                printf("Current state: BRAKING\n");
                break;
            case ERROR:
                printf("Current state: ERROR\n");
                break;
        }
        
        if (command == 0) {
            break; // Exit the loop when the command is 0
        }
    }
    
    return 0;
}
