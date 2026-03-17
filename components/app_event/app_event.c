#include "esp_log.h"
#include "app_event.h"

#define APP_EVENT_TAG "app_event"

/* Event loop configuration */
#define APP_EVENT_QUEUE_SIZE      16
#define APP_EVENT_TASK_PRIORITY   5
#define APP_EVENT_TASK_STACK_SIZE 3072

// Defines the event base for this application's custom events.
ESP_EVENT_DEFINE_BASE(APP_EVENT_BASE);

// Handle for the dedicated application event loop.
static esp_event_loop_handle_t s_loop = NULL;

esp_err_t app_event_init(void)
{
    // Prevent re-initialization.
    if (s_loop != NULL)
    {
        ESP_LOGW(APP_EVENT_TAG, "Already initialized");
        return ESP_OK;
    }

    // Configuration for the new event loop.
    esp_event_loop_args_t args = {
        .queue_size = APP_EVENT_QUEUE_SIZE,
        .task_name = "app_event_task", // Dedicated task for handling events.
        .task_priority = APP_EVENT_TASK_PRIORITY,
        .task_stack_size = APP_EVENT_TASK_STACK_SIZE,
        .task_core_id = tskNO_AFFINITY,
    };

    esp_err_t err = esp_event_loop_create(&args, &s_loop);
    if (err != ESP_OK)
        ESP_LOGE(APP_EVENT_TAG, "Failed to create event loop: %s", esp_err_to_name(err));
    return err;
}

esp_err_t app_event_post_with_timeout(app_event_id_t event_id, const void *event_data,
                                      size_t event_data_size, TickType_t timeout)
{
    if (s_loop == NULL)
    {
        ESP_LOGE(APP_EVENT_TAG, "Event loop not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return esp_event_post_to(s_loop, APP_EVENT_BASE, (int32_t)event_id,
                             event_data, event_data_size, timeout);
}

esp_err_t app_event_post(app_event_id_t event_id, const void *event_data, size_t event_data_size)
{
    return app_event_post_with_timeout(event_id, event_data, event_data_size, portMAX_DELAY);
}

esp_err_t app_event_handler_register(int32_t event_id, esp_event_handler_t event_handler,
                                     void *event_handler_arg)
{
    if (s_loop == NULL)
    {
        ESP_LOGE(APP_EVENT_TAG, "Event loop not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (event_handler == NULL)
        return ESP_ERR_INVALID_ARG;

    return esp_event_handler_register_with(s_loop, APP_EVENT_BASE, event_id,
                                           event_handler, event_handler_arg);
}

esp_err_t app_event_handler_unregister(int32_t event_id, esp_event_handler_t event_handler)
{
    if (s_loop == NULL)
    {
        ESP_LOGE(APP_EVENT_TAG, "Event loop not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (event_handler == NULL)
        return ESP_ERR_INVALID_ARG;

    return esp_event_handler_unregister_with(s_loop, APP_EVENT_BASE, event_id, event_handler);
}
