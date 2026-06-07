//
// Created by zhl on 6/7/26.
//

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>


// Handle for the loop() task
TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;
QueueHandle_t queue;



void Demo_Task(void *arg)
{
    char txBuffer[50];
    queue = xQueueCreate(5, sizeof(txBuffer));
    if (queue == 0)
    {
        printf("Failed to create queue= %p\n", queue);
    }

    sprintf(txBuffer, "Hello from Demo_Task 1");
    xQueueSend(queue, (void*)txBuffer, (TickType_t)0);

    sprintf(txBuffer, "Hello from Demo_Task 2");
    xQueueSend(queue, (void*)txBuffer, (TickType_t)0);

    sprintf(txBuffer, "Hello from Demo_Task 3");
    xQueueSend(queue, (void*)txBuffer, (TickType_t)0);

    while(1){
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
}

void Demo_Task2(void *arg)
{
    char rxBuffer[50];
    while(1){
        if( xQueueReceive(queue, &(rxBuffer), (TickType_t)5))
        {
            printf("Received data from queue == %s/n", rxBuffer);
            vTaskDelay(1000/ portTICK_RATE_MS);

        }
    }
}


void setup() {
    // Inicialización de hardware, sensores, actuadores, comunicación, etc.
    xTaskCreate(Demo_Task, "Demo_Task", 4096, NULL, 10, &myTaskHandle);
    xTaskCreatePinnedToCore(Demo_Task2, "Demo_Task2", 4096, NULL,10, &myTaskHandle2, 1);

    vTaskDelete(NULL); // NULL hace referenica al task "void loop()"
}

void loop() {
    // NO LO USAMOS
}

