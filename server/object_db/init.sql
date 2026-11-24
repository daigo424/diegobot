-- データベースを初めて作るときだけ流れる。表を変えたら、作り直すか手で移す。
CREATE EXTENSION IF NOT EXISTS vector;

-- 見た1回分。埋め込みはモデルごとに次元が違うため、モデル名と版を一緒に持ち、別のモデルの埋め込みとは比べない。
-- 切り出し画像を残しておけば、モデルを替えても埋め込みを作り直せる。
CREATE TABLE observations (
    id          bigserial PRIMARY KEY,
    model       text NOT NULL,
    embedding   vector(384) NOT NULL,
    image_path  text,
    crop_px     real,
    captured_at timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX observations_embedding_idx ON observations USING hnsw (embedding vector_cosine_ops);

-- 見た物が、ある名前(ユーザーの言葉のまま)の物だったかどうか。違った答えも残す。
CREATE TABLE verdicts (
    id             bigserial PRIMARY KEY,
    observation_id bigint NOT NULL REFERENCES observations (id) ON DELETE CASCADE,
    name           text NOT NULL,
    match          boolean NOT NULL,
    source         text NOT NULL DEFAULT 'vlm',
    created_at     timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX verdicts_name_idx ON verdicts (name);
