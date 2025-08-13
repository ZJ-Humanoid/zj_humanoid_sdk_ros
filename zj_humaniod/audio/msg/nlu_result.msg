# nlu_result.msg
string intent          # 识别出的意图标签
float32 confidence     # 置信度 (0.0 - 1.0)
entity[] entities      # 提取的实体列表，类型为 entity
