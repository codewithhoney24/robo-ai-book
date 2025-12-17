class RAGChatbotException(Exception):
    """Base exception for RAG Chatbot application"""
    def __init__(self, message: str, error_code: str = "RAG_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class QueryTooLongException(RAGChatbotException):
    """Raised when a query exceeds the maximum allowed length"""
    def __init__(self, message: str = "The query exceeds the maximum allowed length"):
        super().__init__(message, "QUERY_TOO_LONG")


class InvalidSessionException(RAGChatbotException):
    """Raised when a session ID is invalid or expired"""
    def __init__(self, message: str = "Invalid or expired session"):
        super().__init__(message, "INVALID_SESSION")


class VectorSearchException(RAGChatbotException):
    """Raised when there's an error with vector search operations"""
    def __init__(self, message: str = "Error occurred during vector search"):
        super().__init__(message, "VECTOR_SEARCH_ERROR")


class DatabaseConnectionException(RAGChatbotException):
    """Raised when there's an issue connecting to the database"""
    def __init__(self, message: str = "Failed to connect to the database"):
        super().__init__(message, "DB_CONNECTION_ERROR")


class EmbeddingGenerationException(RAGChatbotException):
    """Raised when there's an error generating embeddings"""
    def __init__(self, message: str = "Error occurred during embedding generation"):
        super().__init__(message, "EMBEDDING_ERROR")