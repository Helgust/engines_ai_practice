#include "math.h"
class tree_elem
{
 public:
     Position m_data;
     tree_elem * m_left;
     tree_elem * m_right;
     tree_elem(Position val)
     {
         m_left = nullptr; // В С++11 лучше использовать nullptr
         m_right = nullptr;
         m_data = val;
     }
};

class binary_tree
{
 private:
    tree_elem * m_root;
    int m_size;
    void delete_tree(tree_elem *);

 public:
    binary_tree(Position);
    ~binary_tree();
    void print();
    bool find(Position);
    void insert(Position);
    void erase(Position);
    int size();
};

binary_tree::binary_tree(Position key)
{
    m_root = new tree_elem(key);
    m_size = 1;
}

binary_tree::~binary_tree()
{
    delete_tree(m_root);
}

void binary_tree::delete_tree(tree_elem * curr)
{
    if (curr)
    {
        delete_tree(curr->m_left);
        delete_tree(curr->m_right);
        delete curr;
    }
}

bool binary_tree::find(Position key)
{
    tree_elem * curr = m_root;
    while (curr && curr->m_data != key)
    {
        if (curr->m_data.F > key.F)
            curr = curr->m_left;
        else
            curr = curr->m_right;
    }
    return curr != nullptr;
}

void binary_tree::insert(Position key)
{
    tree_elem * curr = m_root;
    while (curr && curr->m_data.F != key.F)
    {
        if (curr->m_data.F > key.F && curr->m_left == nullptr)
        {
            curr->m_left = new tree_elem(key);
            ++m_size;
            return;
        }
        if (curr->m_data.F < key.F && curr->m_right == nullptr)
        {
            curr->m_right = new tree_elem(key);
            ++m_size;
            return;
        }
        if (curr->m_data.F > key.F)
            curr = curr->m_left;
        else
            curr = curr->m_right;
    }
}

void binary_tree::erase(Position key)
{
    tree_elem * curr = m_root;
    tree_elem * parent = nullptr;
    while (curr && curr->m_data.F != key.F)
    {
        parent = curr;
        if (curr->m_data.F > key.F)
        {
            curr = curr->m_left;
        }
        else
        {
            curr = curr->m_right;
        }
    }
    if (!curr)
        return;
    if (curr->m_left == nullptr)
    {
        // Вместо curr подвешивается его правое поддерево
        if (parent && parent->m_left == curr)
            parent->m_left = curr->m_right;
        if (parent && parent->m_right == curr)
            parent->m_right = curr->m_right;
        --m_size;
        delete curr;
        return;
    }
    if (curr->m_right == nullptr)
    {
        // Вместо curr подвешивается его левое поддерево
        if (parent && parent->m_left == curr)
            parent->m_left = curr->m_left;
        if (parent && parent->m_right == curr)
            parent->m_right = curr->m_left;
        --m_size;
        delete curr;
        return;
    }
    // У элемента есть два потомка, тогда на место элемента поставим
    // наименьший элемент из его правого поддерева
    tree_elem * replace = curr->m_right;
    while (replace->m_left)
        replace = replace->m_left;
    Position replace_value = replace->m_data;
    erase(replace_value);
    curr->m_data = replace_value;
}
