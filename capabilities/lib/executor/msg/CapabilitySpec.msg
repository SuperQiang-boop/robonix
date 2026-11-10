# LLM-facing capability description. Planner builds one of these per
# (provider_id, contract_id) pair it wants to expose to the model.
string provider_id
string contract_id
string description
string input_schema_json
