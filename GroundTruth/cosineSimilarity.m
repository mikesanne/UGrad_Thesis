function alpha = cosineSimilarity(E, G)

alpha = dot(E,G) / (norm(E) + norm(G));