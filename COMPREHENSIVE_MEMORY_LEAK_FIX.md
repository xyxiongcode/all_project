# 全面的内存泄漏修复

## 发现的问题

1. **compute_loss中模型返回的GPU张量未立即释放**
   - `pred_ng, pred_mg, critic_pred, augment_pred, noise, aux_pred` 在使用后没有被立即删除
   - 这些张量在计算损失后仍然保留在内存中

2. **forward中大量中间变量未清理**
   - `ng_cond_embed_tensor`, `mg_cond_embed_tensor`, `cond_pos_embed` 等
   - `pointgoal_embed`, `nogoal_embed`, `imagegoal_embed`, `pixelgoal_embed`
   - `ng_output`, `mg_output` 等在返回时仍存在计算图引用

3. **cond_critic_mask可能重复创建**
   - 每次forward都检查并可能创建，旧的mask可能没有正确清理

4. **inputs中的张量转换后没有删除原始引用**

## 修复方案

### 1. 在compute_loss中立即删除所有GPU张量
- 计算损失后立即提取标量值
- 删除所有模型返回的GPU张量
- 删除所有输入的GPU张量

### 2. 在forward返回前清理所有中间变量
- 删除所有不再需要的中间变量
- 只保留返回所需的最小变量

### 3. 确保cond_critic_mask只创建一次
- 删除旧的mask后再创建新的
- 避免重复创建导致内存泄漏

### 4. 在每个训练步骤后立即清理
- 删除outputs和inputs
- 清理CUDA缓存
