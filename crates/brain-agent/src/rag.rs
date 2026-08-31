//! RAG（检索增强生成）：给 Agent 加"记忆/知识检索"能力（参考 Rig 的 RAG）。
//!
//! 提供：
//! - `Embedder` trait（文本→向量）+ 确定性 `MockEmbedder`（离线、可测）
//! - `MemoryStore`：文档向量存储 + 余弦相似度检索
//! - `RetrieveTool`：把检索能力包装成 Agent 可调用的工具

use std::collections::HashMap;
use std::sync::Arc;

use brain_core::Result;

use crate::tool::Tool;

/// 文本嵌入器。
pub trait Embedder: Send + Sync {
    fn embed(&self, text: &str) -> Vec<f32>;
}

/// 确定性的离线嵌入器：把文本分词，按词哈希到固定维度向量（袋词风格）。
///
/// 相同词汇 → 相似向量，保证检索测试完全确定。
pub struct MockEmbedder {
    dim: usize,
}

impl MockEmbedder {
    pub fn new(dim: usize) -> Self {
        Self { dim }
    }
}

impl Default for MockEmbedder {
    fn default() -> Self {
        Self::new(64)
    }
}

/// FNV-1a 哈希（稳定、无依赖）。
fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

impl Embedder for MockEmbedder {
    fn embed(&self, text: &str) -> Vec<f32> {
        let mut v = vec![0.0f32; self.dim];
        for tok in text
            .split(|c: char| !c.is_alphanumeric())
            .filter(|t| !t.is_empty())
        {
            let idx = (fnv1a(&tok.to_lowercase()) % self.dim as u64) as usize;
            v[idx] += 1.0;
        }
        // 归一化（零向量保持全零）。
        let norm = v.iter().map(|x| x * x).sum::<f32>().sqrt();
        if norm > 1e-9 {
            for x in v.iter_mut() {
                *x /= norm;
            }
        }
        v
    }
}

/// 一份文档（历史记录/知识）。
#[derive(Debug, Clone)]
pub struct Document {
    pub id: String,
    pub text: String,
    pub embedding: Vec<f32>,
}

/// 内存向量存储：添加文档、按查询检索 Top-k。
pub struct MemoryStore {
    docs: Vec<Document>,
    embedder: Arc<dyn Embedder>,
}

impl MemoryStore {
    pub fn new(embedder: Arc<dyn Embedder>) -> Self {
        Self {
            docs: Vec::new(),
            embedder,
        }
    }

    /// 添加一篇文档。
    pub fn add(&mut self, id: impl Into<String>, text: impl Into<String>) {
        let text = text.into();
        let embedding = self.embedder.embed(&text);
        self.docs.push(Document {
            id: id.into(),
            text,
            embedding,
        });
    }

    /// 检索与查询最相似的 Top-k 文档文本。
    pub fn search(&self, query: &str, k: usize) -> Vec<String> {
        let qv = self.embedder.embed(query);
        let mut scored: Vec<(f32, &Document)> = self
            .docs
            .iter()
            .map(|d| (cosine(&qv, &d.embedding), d))
            .collect();
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        scored
            .into_iter()
            .take(k)
            .map(|(_, d)| d.text.clone())
            .collect()
    }

    /// 文档数量。
    pub fn len(&self) -> usize {
        self.docs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.docs.is_empty()
    }
}

fn cosine(a: &[f32], b: &[f32]) -> f32 {
    let dot: f32 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    dot
}

/// 把检索能力封装成 Agent 工具：参数 `query`，返回 Top-k 相关记录。
pub struct RetrieveTool {
    store: Arc<MemoryStore>,
    top_k: usize,
}

impl RetrieveTool {
    pub fn new(store: Arc<MemoryStore>, top_k: usize) -> Self {
        Self { store, top_k }
    }
}

impl Tool for RetrieveTool {
    fn name(&self) -> &str {
        "retrieve"
    }
    fn description(&self) -> &str {
        "检索历史巡检记录/知识库"
    }
    fn run(&self, args: &HashMap<String, String>) -> Result<String> {
        let query = args.get("query").cloned().unwrap_or_default();
        let hits = self.store.search(&query, self.top_k);
        if hits.is_empty() {
            Ok("未找到相关记录".to_string())
        } else {
            let mut out = String::from("相关记录：\n");
            for (i, h) in hits.iter().enumerate() {
                out.push_str(&format!("  [{i}] {h}\n"));
            }
            Ok(out.trim_end().to_string())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn similar_texts_embed_similarly() {
        let e = MockEmbedder::new(64);
        let a = e.embed("tower three inspection defect");
        let b = e.embed("tower three inspection");
        let c = e.embed("battery voltage low alarm");
        assert!(cosine(&a, &b) > 0.8, "similar texts should be close");
        assert!(cosine(&a, &c) < cosine(&a, &b), "unrelated should differ");
    }

    #[test]
    fn store_retrieves_relevant() {
        let mut store = MemoryStore::new(Arc::new(MockEmbedder::default()));
        store.add("doc1", "tower 3 inspection found no defect");
        store.add("doc2", "battery low alarm on node 7");
        store.add("doc3", "tower 5 insulator cracked");
        let hits = store.search("tower inspection", 2);
        assert!(hits[0].contains("tower 3") || hits[0].contains("tower 5"));
    }

    #[test]
    fn retrieve_tool_through_agent() {
        let store = Arc::new({
            let mut s = MemoryStore::new(Arc::new(MockEmbedder::default()));
            s.add("r1", "tower 3 inspection found no defect");
            s
        });
        let tool = Arc::new(RetrieveTool::new(store, 1));
        let mut args = HashMap::new();
        args.insert("query".to_string(), "tower 3".to_string());
        let out = tool.run(&args).unwrap();
        assert!(out.contains("tower 3"));
    }
}
