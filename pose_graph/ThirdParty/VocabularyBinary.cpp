#include "VocabularyBinary.hpp"
#include <opencv2/core/core.hpp>

SVINLoop::Vocabulary::Vocabulary() : nNodes(0), nodes(nullptr), nWords(0), words(nullptr) {}

SVINLoop::Vocabulary::~Vocabulary() {
  if (nodes != nullptr) {
    delete[] nodes;
    nodes = nullptr;
  }

  if (words != nullptr) {
    delete[] words;
    words = nullptr;
  }
}

void SVINLoop::Vocabulary::serialize(std::ofstream& stream) {
  stream.write((const char*)this, staticDataSize());
  stream.write((const char*)nodes, sizeof(Node) * nNodes);
  stream.write((const char*)words, sizeof(Word) * nWords);
}

void SVINLoop::Vocabulary::deserialize(std::ifstream& stream) {
  stream.read((char*)this, staticDataSize());

  nodes = new Node[nNodes];
  stream.read((char*)nodes, sizeof(Node) * nNodes);

  words = new Word[nWords];
  stream.read((char*)words, sizeof(Word) * nWords);
}
