
topics = ["a", "b", "c", "d","e"]
alpha = {topic: [] for topic in topics}  # Inicializa un diccionario con listas vacías para cada tópico

for i in range(10):
    for topic in topics:
        alpha[topic].append(i)  # Añade 'i' a la lista correspondiente al tópico

print(alpha)