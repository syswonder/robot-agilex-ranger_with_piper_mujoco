config:
  vlm_base_url:
    type: string
    default: https://api.openai.com/v1
    description: OpenAI-compatible API base URL used for ambiguous target matching.
    example: https://api.example.com/v1
    failure: Ambiguous picks fail when the endpoint is unavailable or malformed.
  vlm_api_key:
    type: string
    default: ""
    sensitive: true
    description: Bearer credential supplied through operator environment expansion.
    example: ${VLM_API_KEY}
    failure: Exact object aliases still work, but ambiguous VLM matching is unavailable.
  vlm_model:
    type: string
    default: gpt-5.6-sol
    description: Model name accepted by the configured OpenAI-compatible endpoint.
    example: gpt-5.6-sol
    failure: The skill returns detection_failed when the endpoint rejects the model.
  vlm_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: HTTP timeout for one VLM target-resolution call.
    example: 30
    failure: CMD_INIT fails when non-numeric or non-positive; requests fail on expiry.
  pick_timeout_s:
    type: float
    unit: seconds
    default: 45.0
    range: greater than 0
    description: Maximum wait for terminal physical feedback from the simulator.
    example: 45
    failure: CMD_INIT fails when invalid; a running pick returns timeout on expiry.
