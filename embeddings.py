import os
from openai import OpenAI
from scipy import spatial
import numpy as np

client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY"),
)

words = ["king", "man", "woman", "queen", "prince", "royal",
         "princess", "duchess", "dragon", "feast", "doctor", "pawn", "knight", "dog",
         "duke", "twin", "full", "mattress", "Elizabeth", "pope", "cardinal"]
response = client.embeddings.create(input = words, model = "text-embedding-3-small")

emb = np.array(list(map(lambda d: np.array(d.embedding), response.data)))
i = 1
emb = np.append(emb, [emb[0] - emb[1]*i + emb[2]*i], axis=0)
words.append("king-"+str(i)+"*man+"+str(i)+"*woman")

# cosine distance to all found embeddings
def compare(e1):
    return map(lambda e2: spatial.distance.cosine(e1, e2), emb)

# index of closest embedding (get second smallest value, we expect zero to be smallest)
def index_of(dists):
    return dists.index( sorted(dists)[1] )

# For fun, what's closest to "king"? It's the "king - man + woman".
list(map(lambda d: print(d), compare(emb[ words.index("king") ])))
print(words[index_of(list(compare(emb[ words.index("king")])))])
print("")

# And what's closet to "queen"? Ok, "king - man + woman".
# Have we proven the equation?
list(map(lambda d: print(d), compare(emb[ words.index("queen") ])))
print(words[index_of(list(compare(emb[ words.index("queen")])))])
print("")

# And the output of "king - man + woman" is... "king"
# It doesn't change enough to become a new word.
list(map(lambda d: print(d), compare(emb[ words.index("king-"+str(i)+"*man+"+str(i)+"*woman") ])))
print(words[index_of(list(compare(emb[ words.index("king-"+str(i)+"*man+"+str(i)+"*woman") ])))])
print("")
